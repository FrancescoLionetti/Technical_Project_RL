#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/move_arm.hpp"		// Includes the custom Action interface. Ensure 'ros2_kdl_package' is the correct package name.
#include "geometry_msgs/msg/point.hpp"			// Includes the message type for specifying the target position (geometry_msgs::msg::Point).

// Alias for the full Action interface for convenience.
using MoveArm = ros2_kdl_package::action::MoveArm;
// Alias for the Client Goal Handle. It maintains the state and interaction for a specific goal.
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;		
class TrajectoryActionClient : public rclcpp::Node
{
public:
    explicit TrajectoryActionClient(const rclcpp::NodeOptions & options)
    : Node("trajectory_action_client", options)
    {
    	// Creates the Action client. This establishes the connection to the Action Server.
        this->client_ptr_ = rclcpp_action::create_client<MoveArm>(
            this,
            "move_arm"); // The name of the action defined in the .action file
    }

    void send_goal(double x, double y, double z)		
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {		// Waits for the Action Server to be available for a maximum of 10 seconds.
            RCLCPP_ERROR(this->get_logger(), "Action server non disponibile dopo 10s");
            //rclcpp::shutdown();
            // If the server isn't available, we exit the function.
            return;
        }

	// Create and populate the Goal message
        auto goal_msg = MoveArm::Goal();
        // The coordinates are assigned to the 'target_position' field, which is a geometry_msgs::msg::Point.						
        goal_msg.target_position.x = x;
        goal_msg.target_position.y = y;
        goal_msg.target_position.z = z;

        RCLCPP_INFO(this->get_logger(), "Invio goal: [x: %f, y: %f, z: %f]", x, y, z);

	// Configure options for sending the goal, including the callback functions.
        auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();
        
        // Binds the callback to handle the Goal acceptance/rejection response.
        send_goal_options.goal_response_callback =
            std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
        // Binds the callback to handle intermediate Feedback updates during execution.
        send_goal_options.feedback_callback =
            std::bind(&TrajectoryActionClient::feedback_callback, this, _1, _2);
        // Binds the callback to handle the final result of the action (Succeeded/Aborted/Canceled).
        send_goal_options.result_callback =
            std::bind(&TrajectoryActionClient::result_callback, this, _1);
        
        // Asynchronously sends the goal to the server.
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    // Shared pointer to the ROS 2 Action Client object.
    rclcpp_action::Client<MoveArm>::SharedPtr client_ptr_;


    void goal_response_callback(const GoalHandleMoveArm::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rifiutato dal server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accettato dal server, in attesa del risultato");
        }
    }

    void feedback_callback(
        GoalHandleMoveArm::SharedPtr,
        const std::shared_ptr<const MoveArm::Feedback> feedback)
    {	
    // Prints the Feedback information, in this case, the norm of the position error.
        RCLCPP_INFO(this->get_logger(), "Feedback ricevuto: Errore = %f", feedback->position_error_norm);
    }

    void result_callback(const GoalHandleMoveArm::WrappedResult & result)
    {	
    // Handles the different termination codes of the Goal.
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            // Checks the specific 'success' field within the Result message.
                RCLCPP_INFO(this->get_logger(), "Goal Raggiunto! Successo: %s", result.result->success ? "true" : "false");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal annullato (aborted)");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal cancellato");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Stato sconosciuto");
                break;
        }
        rclcpp::shutdown(); // Esci dopo aver ricevuto il risultato
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Controlla gli argomenti per x, y, z
    if (argc != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Uso: ros2 run ros2_kdl_package	 action_client_node <x> <y> <z>");
        return 1;
    }

    auto action_client = std::make_shared<TrajectoryActionClient>(rclcpp::NodeOptions());
    
    // Converte gli argomenti in double e invia il goal
    try {
        double x = std::stod(argv[1]);
        double y = std::stod(argv[2]);
        double z = std::stod(argv[3]);
        action_client->send_goal(x, y, z);
    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Argomenti non validi. x, y, z devono essere numeri.");
        return 1;
    }

    rclcpp::spin(action_client);
    //rclcpp::shutdown();
    return 0;
}
