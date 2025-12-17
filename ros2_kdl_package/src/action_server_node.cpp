#include <stdio.h> 
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <algorithm> // Per std::min/max

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 

//action interface definition
#include "ros2_kdl_package/action/move_arm.hpp" 
#include "geometry_msgs/msg/point.hpp"

//custom KDL libraries
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

// ACTION INTERFACE AND HANDLE ALIASES
using MoveArm = ros2_kdl_package::action::MoveArm;
// Goal Handle alias for the Action Server side
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;


class TrajectoryActionServer : public rclcpp::Node
{
    public:
        TrajectoryActionServer()
        : Node("action_server_node"),
        node_handle_(std::shared_ptr<TrajectoryActionServer>(this))
        {
           // --- LOAD PARAMETERS AND ROBOT MODEL SETUP ---
            
            // Load control and trajectory parameters
            declare_parameter("cmd_interface", "velocity"); 
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Cmd interface for KDL logic: '%s'", cmd_interface_.c_str());

            // Parametri per la traiettoria
            declare_parameter("traj_duration", 1.5);
            get_parameter("traj_duration", traj_duration);
            declare_parameter("acc_duration", 0.5);
            get_parameter("acc_duration", acc_duration);
            declare_parameter("total_time", 1.5);
            get_parameter("total_time", total_time);
            declare_parameter("trajectory_len", 150);
            get_parameter("trajectory_len", trajectory_len);
            declare_parameter("Kp", 5);
            get_parameter("Kp", Kp);
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            declare_parameter("lambda", 5.0);
            get_parameter("lambda", lambda);
            declare_parameter("ctrl", "velocity_ctrl");
            get_parameter("ctrl", ctrl);

            RCLCPP_INFO(get_logger(), "Parameters loaded: traj_duration: %f, Kp: %d", traj_duration, Kp);

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false;

            // --- CARICAMENTO MODELLO ROBOT (KDL) ---
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "/iiwa/robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // Crea Albero Cinematico
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);
            
            // Setta i limiti 
            unsigned int nj = robot_->getNrJnts();
            q_min_.resize(nj); q_max_.resize(nj);
            q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; 
            q_max_.data <<  2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
            robot_->setJntLimits(q_min_, q_max_);
            
            joint_positions_.resize(nj);
            joint_velocities_.resize(nj);
            joint_positions_cmd_.resize(nj);
            joint_velocities_cmd_.resize(nj);
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();
            desired_commands_.resize(nj);

            // Subscriber a jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/iiwa/joint_states", 10, std::bind(&TrajectoryActionServer::joint_state_subscriber, this, std::placeholders::_1));

            // Attendi il primo messaggio joint_state (stato iniziale)
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "Waiting for joint_states data...");
                rclcpp::spin_some(node_handle_);
            }

            // Aggiorna KDLrobot e crea il controller
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Inizializza controller
            controller = std::make_unique<KDLController>(*robot_); 

            // Configura il publisher (basato sui parametri) <-- FIX 3: Ripristinato il blocco di selezione topic
            if(cmd_interface_ == "position"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            } else if(cmd_interface_ == "velocity"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/velocity_controller/commands", 10);
            } else if(cmd_interface_ == "effort"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            }
            
            // --- CREAZIONE ACTION SERVER ---
            using namespace std::placeholders;
            this->action_server_ = rclcpp_action::create_server<MoveArm>(
                this,
                "move_arm", 
                std::bind(&TrajectoryActionServer::handle_goal, this, _1, _2),
                std::bind(&TrajectoryActionServer::handle_cancel, this, _1),
                std::bind(&TrajectoryActionServer::handle_accepted, this, _1));
            
            RCLCPP_INFO(get_logger(), "Action server 'move_arm' avviato e pronto.");
        }

    private:
        // ... handle_goal, handle_cancel, handle_accepted (Nessuna modifica necessaria) ...
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveArm::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Ricevuto nuovo goal per la traiettoria!");
            (void)uuid;
            if(current_goal_handle_ && current_goal_handle_->is_active())
            {
                RCLCPP_WARN(get_logger(), "Un altro goal è già attivo. Rifiuto.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(get_logger(), "Target: [%f, %f, %f]", goal->target_position.x, goal->target_position.y, goal->target_position.z);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Richiesta di cancellazione ricevuta");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
        {
            std::thread{std::bind(&TrajectoryActionServer::execute_trajectory, this, goal_handle)}.detach();
        }


        // --- Nuovo Thread di Esecuzione (Il cuore del controllo) ---
        void execute_trajectory(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
        {
            this->current_goal_handle_ = goal_handle;
            this->current_goal_ = goal_handle->get_goal();

            t_ = 0.0;
            iteration_ = 0;
            
            if (!joint_state_available_) {
                 RCLCPP_ERROR(get_logger(), "Stato iniziale del giunto non disponibile. Annullamento goal.");
                 auto result = std::make_shared<MoveArm::Result>();
                 result->success = false;
                 goal_handle->abort(result);
                 return;
            }

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame f_start = robot_->getEEFrame(); 
            Eigen::Vector3d init_position(f_start.p.data);

            Eigen::Vector3d end_position;
            end_position << current_goal_->target_position.x,
                            current_goal_->target_position.y,
                            current_goal_->target_position.z;
            
            RCLCPP_INFO(get_logger(), "Inizio traiettoria da [%f, %f, %f] a [%f, %f, %f]",
                        init_position[0], init_position[1], init_position[2],
                        end_position[0], end_position[1], end_position[2]);

            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
            
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            rclcpp::Rate rate(loop_rate);

            joint_positions_cmd_ = joint_positions_; // Inizializza il comando di posizione

            // --- LOOP DI ESECUZIONE (Controllo Cinematico) ---
            
            while (rclcpp::ok() && t_ <= total_time)
            {
                // 1. Gestione Annullamento Cliente
                if (goal_handle->is_canceling()) {
                    publish_zero_commands();
                    auto result = std::make_shared<MoveArm::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal cancellato dal client.");
                    return;
                }

                // 2. Calcolo Traiettoria
                if(s_type_ == "trapezoidal"){
                    p_ = planner_.linear_traj_trapezoidal(t_);
                } else if(s_type_ == "cubic"){
                    p_ = planner_.linear_traj_cubic(t_);
                }

                // 3. Update Robot State
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
                KDL::Frame cartpos = robot_->getEEFrame(); 

                // 4. Calcolo Errore
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(f_start.M), toEigen(cartpos.M)); 
                
                // 5. Pubblicazione Feedback
                auto feedback_msg = std::make_shared<MoveArm::Feedback>();
                feedback_msg->position_error_norm = error.norm();
                goal_handle->publish_feedback(feedback_msg);

                // 6. Legge di Controllo (Usiamo il controllo KDL, che genera velocità)
                if(cmd_interface_ == "velocity")
                {
                    if (ctrl == "velocity_ctrl_null") {
                        // Controllo con null-space (punto 1b)
                        Eigen::VectorXd qdot = controller->velocity_ctrl_null(p_.pos, Kp, lambda);
                        joint_velocities_cmd_.data = qdot;
                    } else {
                        // Controllo standard in velocità
                        Vector6d cartvel;
                        cartvel << p_.vel + Kp * error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                    }
                    
                    // *** NUOVO CHECK DI DEBUG: se NaN, esci immediatamente ***
                    if (joint_velocities_cmd_.data.hasNaN()){ 
                        RCLCPP_ERROR(get_logger(), "!!! ERRORE CINEMATICO: joint_velocities_cmd contiene NaN. Interruzione !!!");
                        publish_zero_commands();
                        break; 
                    }
                    
                    // Esegui il comando
                    publish_commands(joint_velocities_cmd_); // <-- FIX 2: Pubblica la VELOCITÀ
                }
                
                // --- Se il comando fosse in POSIZIONE (NON implementato per KDL) ---
                // Se volessi usare il position command, dovresti fare l'integrazione qui:
                /*
                else if(cmd_interface_ == "position") {
                    // La tua logica di integrazione precedente, che NON è KDL standard
                    for (int i = 0; i < joint_positions_cmd_.data.size(); ++i) {
                        joint_positions_cmd_(i) += joint_velocities_cmd_(i) * dt;
                        joint_positions_cmd_(i) = std::min(std::max(joint_positions_cmd_(i), q_min_(i)), q_max_(i));
                    }
                    publish_positions(joint_positions_cmd_);
                }
                */

                // 7. Incremento temporale e attesa
                t_ += dt;
                rate.sleep();
            }

            // --- FINE TRAIETTORIA ---
            
            publish_zero_commands();

            RCLCPP_INFO(this->get_logger(), "Traiettoria completata. Inizio risultato.");
            
            // Imposta il risultato dell'azione
            auto result = std::make_shared<MoveArm::Result>();
            // Se il loop è uscito per t_ <= total_time, il successo è true, altrimenti NaN/Cancel/Aborted è false
            result->success = (t_ > total_time); 
            if (result->success) {
                goal_handle->succeed(result);
            } else {
                goal_handle->abort(result);
            }
            this->current_goal_handle_ = nullptr;
        }

        // Funzioni helper (le tue funzioni originali, corrette solo nella chiamata)
        void publish_commands(const KDL::JntArray& cmd_data)
        {
             for (long int i = 0; i < cmd_data.data.size(); ++i) {
                desired_commands_[i] = cmd_data(i);
            }
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        void publish_positions(const KDL::JntArray& q_cmd)
        {
            if (!cmdPublisher_) return;

            if (desired_commands_.size() != q_cmd.data.size())
                desired_commands_.resize(q_cmd.data.size());

            for (long i = 0; i < q_cmd.data.size(); ++i)
                desired_commands_[i] = q_cmd(i);

            std_msgs::msg::Float64MultiArray msg;
            msg.data = desired_commands_; 
            cmdPublisher_->publish(msg);
        }

        void publish_zero_commands()
        {
            if (cmd_interface_ == "velocity" || cmd_interface_ == "effort") {
                std::fill(desired_commands_.begin(), desired_commands_.end(), 0.0);
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        
        rclcpp::Node::SharedPtr node_handle_;
        std::vector<double> desired_commands_;
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;
        KDL::JntArray q_min_, q_max_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        trajectory_point p_;
        std::unique_ptr<KDLController> controller;
        std::string ctrl;
        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_ = "linear";
        std::string s_type_;
        
        double traj_duration;
        double acc_duration;
        double total_time;
        int trajectory_len;
        int Kp;
        double lambda;
        
        rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
        std::shared_ptr<GoalHandleMoveArm> current_goal_handle_ = nullptr; 
        std::shared_ptr<const MoveArm::Goal> current_goal_;
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionServer>());
    rclcpp::shutdown();
    return 0; 
}
