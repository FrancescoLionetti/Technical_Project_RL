// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>                                                                  

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            //Fase di caricamento dei parametri del robot
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // default to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            //Declare parameters asked in the homework                                                                                   
            //Traj_duration-durata della traiettoria
            declare_parameter("traj_duration", 1.5);  //REGISTRA UN PARAMETRO CHIAMATO TRAJ_DURATION E GLI DA VALORE DI DEFAULT 1.5
            get_parameter("traj_duration", traj_duration);  //LEGGE IL AVALORE ATTUALE DI TRAJ_DUR E LO AGGIORNA NELLA VARIABILE MEMBBRO
            
            //Acc_duration-durata di accelerazione o decelerazione
            declare_parameter("acc_duration", 0.5);
            get_parameter("acc_duration", acc_duration);
            
            //Total_time-tempo totale di simulazione
            declare_parameter("total_time", 1.5);
            get_parameter("total_time", total_time);
            
            //Trajectory_len-quanto lunga è la linea da seguire
            declare_parameter("trajectory_len", 150);
            get_parameter("trajectory_len", trajectory_len);
            
            //Kp-guadagno proporzionale del controllo di posizione (più alto più aggressivo)
            declare_parameter("Kp", 5);
            get_parameter("Kp", Kp);
            
            //End_position_vec-posizione cartesiana finale
            declare_parameter("end_position_vec", std::vector<double>{}); 
            get_parameter("end_position_vec", end_position_vec);
            
            //Lambda-scaling factor of velocity_ctrl_null command
            declare_parameter("lambda", 5.0);
            get_parameter("lambda", lambda);
            
            //Ctrl command
            declare_parameter("ctrl", "velocity_ctrl");
	    get_parameter("ctrl", ctrl);

            
            //Print messages
            RCLCPP_INFO(get_logger(), "Parameters loaded:");  
            RCLCPP_INFO(get_logger(), "  traj_duration: %f", traj_duration);   //STAMPA A VIDEO IL VALORE CHE IL NODO STA LEGGENDO
            RCLCPP_INFO(get_logger(), "  acc_duration: %f", acc_duration);
            RCLCPP_INFO(get_logger(), "  total_time: %f", total_time);
            RCLCPP_INFO(get_logger(), "  trajectory_len: %d", trajectory_len);
            RCLCPP_INFO(get_logger(), "  Kp: %d", Kp);
            if(cmd_interface_ == "velocity"){
            RCLCPP_INFO(get_logger(), "  ctrl: %s", ctrl.c_str());
            	if (ctrl == "velocity_ctrl_null") {
                    	RCLCPP_INFO(get_logger(),"  lambda: %f", lambda);
    		}  
            }
            if(end_position_vec.size() == 3) {
                 RCLCPP_INFO(get_logger(), "  end_position_vec: [%f, %f, %f]", 
                    end_position_vec[0], end_position_vec[1], end_position_vec[2]);
            } else {
                 RCLCPP_WARN(get_logger(), "  end_position_vec: not set or invalid size. Will use default logic.");
            }
            
            
            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            //CARICAMENTO MODELLO ROBOT
            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            //GENERA ALBERO CINEMATICO
            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            //IL CODICE ATTENDE DI RICEVERE MESSAGGIO DAL TOPIC JOINT STATES PER CAPIRE LA POSIZIONE DI PARTENZA DEL ROBOT
            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);   //SE LASCIAMO SOLO QUESTO VIENE DISTRUTTO APPENA IL COSTRUTTORE FINISCE
	    controller = std::make_unique<KDLController>(*robot_);

            //FASE DI SETUP DEL PIANIFICATORE
            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            //Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];                                       

            Eigen::Vector3d end_position;
            if (end_position_vec.size() == 3) {
                end_position << end_position_vec[0], end_position_vec[1], end_position_vec[2];   //attenzione a non fare shadowing tra il vettore che passi dallo yaml e il tipo eigen vector
            } else {
                // Fallback alla logica originale se il parametro non è impostato correttamente
                RCLCPP_WARN(get_logger(), "Using original relative logic for end_position.");
                end_position << init_position[0], -init_position[1], init_position[2];
            }
            
            
            // Plan trajectory
            //double traj_duration = 1.5, acc_duration = 0.5; 
            double traj_radius = 0.15;

            // Retrieve the first trajectory point
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }
            // // Retrieve the first trajectory point
            // trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            
            //IN BASE AL CMD_INTERFACE CREA UN PUBLISHER SUL TOPIC CORRETTO DEL CONTROLLO
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),      //TIMER CHE CHIAMA LA FUNZIONE PUBLISHER OGNI 100 MS
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:
        //FUNZIONE RICHIAMATA DAL TIMER OGNI 100 MS
        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            //double total_time = 1.5; // 
            //int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            //int Kp = 5;
            t_+=dt;

            //CONTROLLO SULLATRAIETTORIA
            if (t_ < total_time){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           //POSIZIONE ATTUALE END EFFECTOR

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos);  //POSIZIONE DESIDERATA END EFFECTOR

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));   //CALCOLA ERRORE TRA POSA DESIDERATA E ATTUALE
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){   //CALCOLA UNA POSIZIONE TARGET PER IL PROSSIMO STEP E USA LA CINEMATICA INVERSA PER RAGGIUNGERLA STABILENDO LE POS DEI GIUNTI
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    
                    if (ctrl == "velocity_ctrl_null") {
                    	// NUOVO controllo con null-space (punto 1b)
        	    	Eigen::VectorXd qdot = controller->velocity_ctrl_null(p_.pos, static_cast<double>(Kp), lambda);
        		joint_velocities_cmd_.data = qdot;
    		    } 
    		     else {
       			 // VECCHIO controllo in velocità (usato nel punto 1a)
        		Vector6d cartvel; 
        		cartvel << p_.vel + Kp * error, o_error;
        		joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
    			}  
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{   //ENTRIAMO IN QUESTA SEZIONE QUANDO ABBIAMO RAGGIUNTO LA DURATA TOTALE DELLA TRAIETTORIA
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;   //SMETTE DI CALCOLARE NUOVI COMANDI E MANDA ZERO COME VALORE DI VEL
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        //FUNZIONE ESEGUITA OGNI VOLTA CHE ARRIVA UN NUOVO MESSAGGIO SUL TOPIC JOINT-STATES. AGGIORNA LE VARIABILI POS GIUNTI E VEL GIUNTI CON I DATI REALI PROVENIENTI DAL ROBOT
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        trajectory_point p_;

        std::unique_ptr<KDLController> controller;
	std::string ctrl;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;

        KDL::Frame init_cart_pose_;
        
        // Variabili membro per i parametri                                                                                      
        double traj_duration;
        double acc_duration;
        double total_time;
        int trajectory_len;
        int Kp;
        std::vector<double> end_position_vec;
        double lambda;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
