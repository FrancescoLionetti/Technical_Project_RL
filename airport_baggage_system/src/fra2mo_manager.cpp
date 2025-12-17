#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <cmath>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Stati della missione
enum class MissionState {
    IDLE,
    NAV_TO_TARGET,      // Verso il cubo (Solo Nav2, niente ArUco)
    DROPPING,           // Scarico
    NAV_TO_HOME,        // Ritorno (Nav2 + Controllo ArUco attivo)
    DOCKING_HOME        // Avvicinamento fine con memoria
};

class Fra2moManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Fra2moManager() : Node("fra2mo_manager")
    {
        // --- 1. COMUNICAZIONE ---
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/fra2mo/status", 10);
        pose_exchange_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fra2mo/exchange_pose", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/fra2mo/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/fra2mo/odometry", 10, std::bind(&Fra2moManager::odom_callback, this, _1));

        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_tag/aruco_tag/pose", 10, std::bind(&Fra2moManager::aruco_pose_callback, this, _1));

        goal_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/fra2mo/goal_input", 10, std::bind(&Fra2moManager::goal_callback, this, _1));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/airport_baggage_world/set_pose");

        // --- 2. MAGNETI ---
        attach_red_pub_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/attach_red", 10);
        detach_red_pub_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/detach_red", 10);
        attach_green_pub_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/attach_green", 10);
        detach_green_pub_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/detach_green", 10);

        // --- 3. TIMER CONTROLLO (MEMORIA) ---
        
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&Fra2moManager::control_loop, this));

        // --- 4. POSE ---
        home_pose_.header.frame_id = "map";
        home_pose_.pose.position.x = -8.35; 
        home_pose_.pose.position.y = 4.22;
        home_pose_.pose.orientation.w = 1.0; 

        current_pose_ = home_pose_; 
        red_pose_.header.frame_id = "map";
        red_pose_.pose.position.x = 8.00; red_pose_.pose.position.y = -4.00; red_pose_.pose.orientation.w = 1.0;
        green_pose_.header.frame_id = "map";
        green_pose_.pose.position.x = 8.00; green_pose_.pose.position.y = 4.00; green_pose_.pose.orientation.w = 1.0;

        mission_state_ = MissionState::IDLE;
        tag_visible_ = false;
        last_tag_time_ = this->now();

        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2)); 
            RCLCPP_WARN(this->get_logger(), "!!! SAFETY RESET: Stacco tutti i cubi !!!");
            for(int i=0; i<3; i++) {
                control_cargo("red", false); control_cargo("green", false);
                std::this_thread::sleep_for(100ms);
            }
            publish_status("ready");
            RCLCPP_INFO(this->get_logger(), "Fra2Mo Manager Pronto.");
        }).detach();
    }

private:
    // --- CALLBACK ARUCO ---
    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_aruco_pose_ = *msg;
        tag_visible_ = true;
        last_tag_time_ = this->now();
        
        // INTERRUZIONE NAV2: Solo se stiamo tornando a casa (NAV_TO_HOME)
        if (mission_state_ == MissionState::NAV_TO_HOME) {
            double dist = msg->pose.position.z;
            // Se il tag è ragionevolmente vicino (< 2.5m), attiviamo il docking
            if (dist < 2.5) {
                RCLCPP_INFO(this->get_logger(), ">>> TAG HOME VISTO! INTERROMPO NAV2 e passo al DOCKING!");
                nav_client_->async_cancel_all_goals(); 
                mission_state_ = MissionState::DOCKING_HOME;
            }
        }
    }

    // --- CONTROL LOOP (LOGICA CON MEMORIA) ---
    void control_loop()
    {
        // 1. Gestione Timeout Memoria (2 secondi)
        if ((this->now() - last_tag_time_).seconds() > 2.0) {
            tag_visible_ = false;
        }

        // 2. Logica attiva solo durante il Docking
        if (mission_state_ == MissionState::DOCKING_HOME) 
        {
            auto cmd = geometry_msgs::msg::Twist();

            // CASO A: Tag perso -> RECOVERY 
            if (!tag_visible_) {
                double search_rot_speed = 0.3; 
                double last_known_lateral = last_aruco_pose_.pose.position.x;
                // Ruota verso l'ultima posizione nota
                cmd.angular.z = (last_known_lateral > 0) ? -search_rot_speed : search_rot_speed;
                cmd_vel_pub_->publish(cmd);
                return; 
            }

            //Tag visibile (o in memoria) -> PID
            double current_dist = last_aruco_pose_.pose.position.z;
            double lateral_error = last_aruco_pose_.pose.position.x;
            
            double target_distance = 0.15; // Il tuo target originale
            double error_distance = current_dist - target_distance;

            
            if (std::abs(error_distance) < 0.02 && std::abs(lateral_error) < 0.02) {
                RCLCPP_INFO(this->get_logger(), ">>> DOCKING HOME COMPLETATO!");
                cmd.linear.x = 0; cmd.angular.z = 0;
                cmd_vel_pub_->publish(cmd);
                
                mission_finished(); 
                return;
            }

            // Calcolo PID
            double kp_dist = 0.25; 
            double kp_ang = 1.0; 

           
            if (std::abs(error_distance) > 0.02) {
                cmd.linear.x = kp_dist * error_distance;
                
                if (cmd.linear.x > 0.20) cmd.linear.x = 0.20;
                else if (cmd.linear.x < -0.20) cmd.linear.x = -0.20;
                else if (std::abs(cmd.linear.x) < 0.05) cmd.linear.x = (cmd.linear.x > 0) ? 0.05 : -0.05;
            }

            // Angular
            cmd.angular.z = -kp_ang * lateral_error;
            cmd.angular.z = std::clamp(cmd.angular.z, -0.4, 0.4);

            cmd_vel_pub_->publish(cmd);
        }
    }

    void goal_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (mission_state_ != MissionState::IDLE) {
            RCLCPP_WARN(this->get_logger(), "Occupato. Ignoro comando.");
            return;
        }

        std::string command = msg->data;
        current_cargo_ = command; 
        RCLCPP_INFO(this->get_logger(), "Ricevuto comando: %s", command.c_str());

        if (command == "red" || command == "green") {
            control_cargo(command, true); 
            std::this_thread::sleep_for(500ms); 
            
            
            geometry_msgs::msg::PoseStamped target = (command == "red") ? red_pose_ : green_pose_;
            mission_state_ = MissionState::NAV_TO_TARGET;
            send_nav_goal(target);
        }
    }

    
    void on_nav_complete(bool success) 
    {
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Navigazione fallita/cancellata.");
            
            if (mission_state_ != MissionState::DOCKING_HOME) {
                control_cargo(current_cargo_, false);
                mission_state_ = MissionState::IDLE;
            }
            return;
        }

        
        if (mission_state_ == MissionState::NAV_TO_TARGET) {
            perform_drop_sequence();
        }
        
        else if (mission_state_ == MissionState::NAV_TO_HOME) {
            RCLCPP_WARN(this->get_logger(), "Arrivato a HOME (Nav2) senza aver visto il Tag.");
            mission_finished();
        }
    }

    void perform_drop_sequence()
    {
        mission_state_ = MissionState::DROPPING;
        RCLCPP_INFO(this->get_logger(), ">>> Arrivato Drop Zone. Scarico...");
        
        geometry_msgs::msg::PoseStamped drop_pose = (current_cargo_ == "red") ? red_pose_ : green_pose_;

        control_cargo(current_cargo_, false);
        std::this_thread::sleep_for(500ms);
        teleport_cargo_to_ground(current_cargo_, drop_pose);

        std::this_thread::sleep_for(2s);
        RCLCPP_INFO(this->get_logger(), ">>> Ritorno verso HOME. Attivo ricerca Tag...");
        
        // Imposto lo stato per il ritorno: QUI si attiverà ArUco se visto
        mission_state_ = MissionState::NAV_TO_HOME;
        send_nav_goal(home_pose_);
    }

    void mission_finished()
    {
        RCLCPP_INFO(this->get_logger(), ">>> MISSIONE CONCLUSA.");
        mission_state_ = MissionState::IDLE;
        current_cargo_ = "";
        publish_current_pose();
        publish_status("home"); 
    }

    void send_nav_goal(geometry_msgs::msg::PoseStamped target)
    {
        if (!nav_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 non disponibile!"); 
            mission_state_ = MissionState::IDLE; 
            return;
        }
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
            
            if (result.code == rclcpp_action::ResultCode::CANCELED) return;
            
            this->on_nav_complete(success);
        };
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Helper Functions invarate
    void publish_status(std::string status) {
        auto msg = std_msgs::msg::String(); msg.data = status; status_pub_->publish(msg);
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.header = msg->header; current_pose_.pose = msg->pose.pose;
    }
    void publish_current_pose() {
        current_pose_.header.frame_id = "map"; pose_exchange_pub_->publish(current_pose_);
    }
    void control_cargo(std::string color, bool attach) {
        auto msg = std_msgs::msg::Empty();
        if (color == "red") (attach ? attach_red_pub_ : detach_red_pub_)->publish(msg);
        else (attach ? attach_green_pub_ : detach_green_pub_)->publish(msg);
    }
    void teleport_cargo_to_ground(std::string color, geometry_msgs::msg::PoseStamped drop_location) {
        std::string model_name = (color == "red") ? "Red_Cube" : "Green_Cube";
        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = model_name;
        request->pose.position.x = drop_location.pose.position.x + 0.5; 
        request->pose.position.y = drop_location.pose.position.y;
        request->pose.position.z = 0.03; 
        request->pose.orientation.w = 1.0;
        if (set_pose_client_->service_is_ready()) set_pose_client_->async_send_request(request);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_exchange_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_; 
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_red_pub_, detach_red_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_green_pub_, detach_green_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_; 

    geometry_msgs::msg::PoseStamped home_pose_, red_pose_, green_pose_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped last_aruco_pose_;
    
    MissionState mission_state_;
    bool tag_visible_;
    rclcpp::Time last_tag_time_;
    std::string current_cargo_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fra2moManager>());
    rclcpp::shutdown();
    return 0;
}
