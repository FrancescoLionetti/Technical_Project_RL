#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <map>
#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros2_kdl_package/action/move_arm.hpp"

using MoveArm = ros2_kdl_package::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;
using namespace std::chrono_literals;

class IIWAManager : public rclcpp::Node
{
public:
    IIWAManager() : Node("iiwa_manager")
    {
        // --- 1. SETUP COORDINATE ---
        pick_positions_["red"] = { -9.18, 4.80, 0.42 }; 
        pick_positions_["green"] = { -8.82, 4.80, 0.42 };
        
        home_position_ = { -8.55, 4.22, 0.80 };

        // Default Pose
        actual_fra2mo_pose_ = { -8.35, 4.22, 0.0 }; 

        detected_["red"] = false;
        detected_["green"] = false;

        // --- 2. SUBSCRIBERS & PUBLISHERS ---
        color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/iiwa/detected_color", 10, std::bind(&IIWAManager::color_callback, this, std::placeholders::_1));

        fra2mo_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/fra2mo/status", 10, std::bind(&IIWAManager::fra2mo_status_callback, this, std::placeholders::_1));

        // Ascolto la posizione di Fra2Mo
        fra2mo_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fra2mo/exchange_pose", 10, std::bind(&IIWAManager::fra2mo_pose_callback, this, std::placeholders::_1));

        fra2mo_pub_ = this->create_publisher<std_msgs::msg::String>("/fra2mo/goal_input", 10);
        
        attach_red_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/attach_red", 10);
        detach_red_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/detach_red", 10);
        attach_green_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/attach_green", 10);
        detach_green_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/detach_green", 10);

        // --- 3. CLIENTS ---
        action_client_ = rclcpp_action::create_client<MoveArm>(this, "move_arm");
        set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/airport_baggage_world/set_pose");

        // --- 4. SAFETY STARTUP ---
        std::thread([this]() {
            std::this_thread::sleep_for(3s);
            RCLCPP_WARN(this->get_logger(), "--- SAFETY: Forcing DETACH on all cubes ---");
            auto msg = std_msgs::msg::Empty();
            for(int i=0; i<3; i++) {
                detach_red_pub_->publish(msg);
                detach_green_pub_->publish(msg);
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "IIWA Manager Ready.");
        }).detach();

        timer_ = this->create_wall_timer(
            1s, [this]() { this->check_and_start(); });
    }

private:
    struct Point3D { double x, y, z; };
    
    
    void fra2mo_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_WARN(this->get_logger(), ">> UPDATE DA FRA2MO RICEVUTO: [x: %.3f, y: %.3f]", 
            actual_fra2mo_pose_.x, actual_fra2mo_pose_.y);
    }

    void color_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string color = msg->data;
        if (color == "red" && !detected_["red"]) {
            RCLCPP_INFO(this->get_logger(), "DETECTED: Red Cube");
            detected_["red"] = true;
        }
        else if (color == "green" && !detected_["green"]) {
            RCLCPP_INFO(this->get_logger(), "DETECTED: Green Cube");
            detected_["green"] = true;
        }
    }

    void fra2mo_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (waiting_for_fra2mo_ && (msg->data == "home" || msg->data == "ready")) {
            RCLCPP_INFO(this->get_logger(), ">>> Fra2Mo TORNATO. Riprendo missione.");
            waiting_for_fra2mo_ = false;
            obj_idx_++;
            current_step_++;
            process_next_goal();
        }
    }

    void check_and_start()
    {
        if (routine_started_) return;
        if (detected_["red"] || detected_["green"]) {
            routine_started_ = true;
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), ">>> AVVIO ROUTINE COMPLETA <<<");
            objects_sequence_ = {"red", "green"};
            start_routine();
        }
    }

    void start_routine()
    {
        obj_idx_ = 0;
        Point3D dummy_drop = { -8.35, 4.22, 0.65 };

        for (const auto& obj_color : objects_sequence_) {
            Point3D target = pick_positions_[obj_color];
            double travel_z = 0.75; 

            // Pick Sequence
            task_queue_.push_back({target.x, target.y, travel_z}); task_types_.push_back("APPROACH_PICK");
            task_queue_.push_back({target.x, target.y, target.z}); task_types_.push_back("PICK_DOWN");
            task_queue_.push_back({target.x, target.y, travel_z}); task_types_.push_back("PICK_UP");

            // Drop Sequence 
            task_queue_.push_back({dummy_drop.x, dummy_drop.y, travel_z}); task_types_.push_back("APPROACH_DROP");
            task_queue_.push_back({dummy_drop.x, dummy_drop.y, dummy_drop.z}); task_types_.push_back("DROP_DOWN");

            // Rest Sequence
            task_queue_.push_back({home_position_.x, home_position_.y, home_position_.z}); task_types_.push_back("GO_REST");
        }
        current_step_ = 0;
        process_next_goal();
    }

    void process_next_goal()
    {
        if (current_step_ >= task_queue_.size()) {
            RCLCPP_INFO(this->get_logger(), "MISSIONE COMPLETATA! Tutti i cubi gestiti.");
            return;
        }
        
        Point3D target = task_queue_[current_step_];
        std::string type = task_types_[current_step_];

        // --- CORREZIONE DINAMICA ---
        if (type == "APPROACH_DROP" || type == "DROP_DOWN") {
            target.x = actual_fra2mo_pose_.x;
            target.y = actual_fra2mo_pose_.y;
            RCLCPP_INFO(this->get_logger(), ">>> DINAMICO: Vado verso Fra2Mo a %.3f, %.3f", target.x, target.y);
        }

        RCLCPP_INFO(this->get_logger(), "Eseguendo step %lu/%lu: %s", current_step_ + 1, task_queue_.size(), type.c_str());
        send_arm_goal(target, type);
    }

    void send_arm_goal(Point3D target, std::string type)
    {
        if (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }
        auto goal_msg = MoveArm::Goal();
        goal_msg.target_position.x = target.x;
        goal_msg.target_position.y = target.y;
        goal_msg.target_position.z = target.z;
        auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this, type, target](const GoalHandleMoveArm::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    handle_task_completion(type, target);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Action failed for type: %s", type.c_str());
                }
            };
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void handle_task_completion(std::string type, Point3D current_pos)
    {
        std::string current_color = objects_sequence_[obj_idx_];
        std::string cube_name = (current_color == "red") ? "Red_Cube" : "Green_Cube";

        if (type == "PICK_DOWN") {
            snap_to_gripper(cube_name, current_pos);
            control_gripper(current_color, true); 
        }
        else if (type == "DROP_DOWN") {
            control_gripper(current_color, false); 
            snap_to_drop(cube_name); 
        }
        else if (type == "GO_REST") {
            RCLCPP_INFO(this->get_logger(), ">>> ORDINE A FRA2MO: %s", current_color.c_str());
            auto msg = std_msgs::msg::String();
            msg.data = current_color;
            fra2mo_pub_->publish(msg);
            waiting_for_fra2mo_ = true;
            return; 
        }
        current_step_++;
        process_next_goal();
    }

    void snap_to_gripper(std::string cube_name, Point3D effector_pos)
    {
        double cube_z_center = effector_pos.z - 0.025; 
        call_set_pose_service(cube_name, effector_pos.x, effector_pos.y, cube_z_center);
        std::this_thread::sleep_for(100ms); 
    }

    void snap_to_drop(std::string cube_name)
    {
        // 1. Coordinate dinamiche da Fra2Mo
        double fra2mo_x = actual_fra2mo_pose_.x;
        double fra2mo_y = actual_fra2mo_pose_.y;
        
        // 2. Offset posteriore
        double drop_x = fra2mo_x + 0.09; 
        double drop_y = fra2mo_y;
        
        // 3. Altezza
        double drop_z = 0.28; 

        call_set_pose_service(cube_name, drop_x, drop_y, drop_z);
        RCLCPP_INFO(this->get_logger(), "MAGIC HANDOVER: %s su Fra2Mo REALE (Base: %.3f, %.3f)", 
            cube_name.c_str(), fra2mo_x, fra2mo_y);
        
        std::this_thread::sleep_for(500ms);
    }

    void call_set_pose_service(std::string entity_name, double x, double y, double z)
    {
        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = entity_name;
        request->pose.position.x = x;
        request->pose.position.y = y;
        request->pose.position.z = z;
        request->pose.orientation.w = 1.0; 

        if (set_pose_client_->service_is_ready()) {
            set_pose_client_->async_send_request(request);
        } else {
            RCLCPP_WARN(this->get_logger(), "Service SetEntityPose non disponibile!");
        }
    }

    void control_gripper(std::string color, bool attach)
    {
        auto msg = std_msgs::msg::Empty();
        if (color == "red") {
            if (attach) attach_red_pub_->publish(msg);
            else detach_red_pub_->publish(msg);
        } else {
            if (attach) attach_green_pub_->publish(msg);
            else detach_green_pub_->publish(msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_INFO(this->get_logger(), "GRIPPER: %s -> %s", color.c_str(), attach ? "ATTACH" : "DETACH");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fra2mo_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fra2mo_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fra2mo_pub_;
    rclcpp_action::Client<MoveArm>::SharedPtr action_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_red_pub_, detach_red_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_green_pub_, detach_green_pub_;
    
    std::map<std::string, bool> detected_;
    std::map<std::string, Point3D> pick_positions_;
    Point3D home_position_;
    Point3D actual_fra2mo_pose_; 
    
    std::vector<std::string> objects_sequence_;
    std::vector<Point3D> task_queue_;
    std::vector<std::string> task_types_;
    size_t current_step_ = 0;
    size_t obj_idx_ = 0;
    bool routine_started_ = false;
    bool waiting_for_fra2mo_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IIWAManager>());
    rclcpp::shutdown();
    return 0;
}
