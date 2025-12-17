// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
// Modificato per il controllo visivo basato su Aruco
// VERSIONE: Aderisce alla logica di calcolo del Jacobiano dell'esempio

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <Eigen/Dense>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class VisionControlNode : public rclcpp::Node
{
    public:
        VisionControlNode()
        : Node("ros2_vision_control_node"), 
        node_handle_(std::shared_ptr<VisionControlNode>(this))
        {
            // Declare and Get ROS Parameters
            declare_parameter("cmd_interface", "velocity"); 
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (cmd_interface_ != "velocity")
            {
                RCLCPP_ERROR(get_logger(),"This control mode only supports 'velocity'!"); 
                rclcpp::shutdown();
                return;
            }

            declare_parameter("ctrl_mode", "vision");
            get_parameter("ctrl_mode", ctrl_mode_);
            RCLCPP_INFO(get_logger(),"Current control mode is: '%s'", ctrl_mode_.c_str());
            
            if (ctrl_mode_ == "vision")
            {
                RCLCPP_INFO(get_logger(),"Controllo visione attivo"); 
            }

            //Initialize State Flags
            aruco_pose_available_ = false; 
            joint_state_available_ = false;
            //Initialize the timestamp for the pose timeout logic
            last_pose_time_ = this->get_clock()->now();

            //Setup KDL Robot from robot_description
            //Create a parameter client to get the robot description from robot_state_publisher
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
                rclcpp::shutdown();
                return;
            }
            
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Get joint count and set joint limits
            nj_ = robot_->getNrJnts();
            KDL::JntArray q_min(nj_), q_max(nj_);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;      
            robot_->setJntLimits(q_min,q_max);   
              
            // Resize KDL arrays       
            joint_positions_.resize(nj_); 
            joint_velocities_.resize(nj_); 
            joint_velocities_cmd_.resize(nj_); 
            joint_velocities_cmd_.data.setZero();

            //Setup Subscribers and Wait for Initial State
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));

            // Wait until the first /joint_states message is received
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            //Initial Robot and Controller Setup
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            
            // Initialize the controller class
            controller_ = std::make_shared<KDLController>(*robot_);

            //Setup Publisher and Control Loop Timer
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            
            // Create a 10Hz timer (100ms) that calls the cmd_publisher function
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Send an initial zero-velocity command to ensure the robot is stationary
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            
            //Setup Vision Subscriber
            image_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&VisionControlNode::imageCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Starting vision control execution (LOGICA ESEMPIO)...");
        }

    private:

        void cmd_publisher(){

            //// This prevents "ghost movement" if the marker hasn't been seen for a while.
            rclcpp::Time now = this->get_clock()->now();
            if ((now - last_pose_time_).seconds() > 0.5) 
            {
                // If last pose is older than 0.5s, mark it as unavailable
                aruco_pose_available_ = false;
            }

            // Update the model with the latest joint positions/velocities
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Get the pose and Jacobian of the DEFAULT End-Effector
            KDL::Frame cartpos = robot_->getEEFrame();           
            KDL::Jacobian J_cam = robot_->getEEJacobian(); 

            // Main Control Logic
            if (ctrl_mode_ == "vision")
            {
                // Check if the marker pose is still valid
                if (!aruco_pose_available_) {
                    RCLCPP_WARN_ONCE(get_logger(), "Aruco marker pose not available or timed out. Stopping.");
                    joint_velocities_cmd_.data.setZero();
                } else {
                    
                    //This logic calculates T_base_cam and J_base_cam in every cycle.
                    // Define the static transformation from the EE to the Camera
                    KDL::Frame T_ee_cam = KDL::Frame(
                        KDL::Rotation::RotY(-1.57) * KDL::Rotation::RotZ(3.14)
                    );
                    
                    // Current camera pose in the base frame: T_base_cam = T_base_ee * T_ee_cam
                    KDL::Frame cartpos_camera = cartpos * T_ee_cam;

                    // Transform the EE Jacobian (J_cam) to the Camera Jacobian (J_cam_camera)
                    // J_cam_camera = R_base_cam * J_cam_base
                    // This gives us the Jacobian of the camera frame, expressed in the base frame.
                    KDL::Jacobian J_cam_camera(nj_);
                    KDL::changeBase(J_cam, cartpos_camera.M, J_cam_camera);
                    
                    Eigen::VectorXd q0_dot = Eigen::VectorXd::Zero(nj_);      

                    // Calculate joint velocities using the vision controller
                    joint_velocities_cmd_.data = controller_->vision_ctrl(
                        pose_in_camera_frame_, // T_cam_aruco
                        cartpos_camera,        // T_base_cam
                        J_cam_camera,          // J_base_cam
                        q0_dot
                    );
                }
            }
            else
            {
                // If control mode is not 'vision', log a warning and send zero velocities
                RCLCPP_WARN_ONCE(get_logger(), "Control mode '%s' not recognized. Sending zero velocities.", ctrl_mode_.c_str());
                joint_velocities_cmd_.data.setZero();
            }


            // Copy the calculated KDL joint velocities to the output vector
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_cmd_(i);
            }

            // Create the message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        // allback for the Aruco marker pose topic (/aruco_single/pose)
        void imageCallback(const geometry_msgs::msg::PoseStamped& msg) {
            aruco_pose_available_ = true;
            // Update the timestamp every time a new pose is received
            last_pose_time_ = this->get_clock()->now();
            
            // Extract position and orientation from the message
            const auto position = msg.pose.position;
            const auto orientation = msg.pose.orientation;

            // Convert from geometry_msgs to KDL types
            KDL::Vector kdl_position(position.x, position.y, position.z);
            KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            );

            // Store the pose (T_cam_aruco) in the class member
            pose_in_camera_frame_.M = kdl_rotation;
            pose_in_camera_frame_.p = kdl_position;
        }

        //Callback for the robot's joint state topic (/joint_states)
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // ROS 2 Members
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr image_sub_; 
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Node::SharedPtr node_handle_;

        // KDL and State Members
        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_; 

        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_velocities_cmd_;

        KDL::Frame pose_in_camera_frame_; // T_cam_aruco

        // State Variables and Parameters
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        unsigned int nj_; 
        bool joint_state_available_;
        bool aruco_pose_available_; 
        std::string cmd_interface_;
        std::string ctrl_mode_; 
        
        // FIX ESSENZIALE: Membro per il timestamp
        rclcpp::Time last_pose_time_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 1;
}
