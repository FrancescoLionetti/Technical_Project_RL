import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import TimerAction

def generate_launch_description(): 

    #Load Robot Description (XACRO)
    # Build the complete path to the xacro file
    xacro_file_name = "iiwa.config.xacro"
    xacro = os.path.join(
        get_package_share_directory("iiwa_description"), "config", xacro_file_name)

    #Define parameters for nodes.
    params={"robot_description": Command(["xacro ", xacro])}

    # Declare a launch argument named 'gz_args' to specify which world to load
    declared_arguments = [
        DeclareLaunchArgument("gz_args",
            default_value=["-r ", PathJoinSubstitution([get_package_share_directory("iiwa_description"), "gazebo", "worlds", "aruco.world"])],
            description="Arguments for gz_sim",
        )
    ]
    
    # This includes the standard launch file from the ros_gz_sim package
    # It starts the Gazebo simulator using the parameters defined above
    gz_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch",                    
                                      'gz_sim.launch.py'])]),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args"),
                              "publish_clock": "true",
                              }.items()
    )

    # Define and configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params,
                   {"use_sim_time": True},  #to synchronize gazebo time and ros2 time
            ],
    )

    # This node calls the `create` executable from ros_gz_sim
    # (read from the '/robot_description' topic) into the running Gazebo simulation.
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "iiwa"],
    )

    # Define the node that will spawn the 'joint_state_broadcaster'
    # This controller publishes joint states for all interfaces.
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Define the node that will spawn the 'velocity_controller'
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Create an event handler that waits for 'spawn_entity_node' to exit
    spawn_jsb_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node, 
            on_exit=[joint_state_broadcaster_node],
        )
    )

    # Create another handler that waits for 'joint_state_broadcaster_node' to exit
    spawn_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[robot_controller_spawner],
        )
    )
    
    # Clock bridge to keep simulation time and ROS2 time synchronized
    clock_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    
    # Camera bridge
    bridge_camera = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            # Bridge Gazebo's '/camera' topic to a ROS 2 'sensor_msgs/msg/Image'
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            # Bridge the '/camera_info' topic as well
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "--ros-args",
            # Remap the ROS 2 topic '/camera' to '/stereo/left/image_rect_color'
            "-r",
            "/camera:=/stereo/left/image_rect_color",
            # Remap the ROS 2 topic '/camera_info' to '/stereo/left/camera_info'
            "-r",
            "/camera_info:=/stereo/left/camera_info",
            # These remappings are likely to match the input topics expected by 'aruco_ros'
        ],
        output="screen",
    )
    
    # Node to visualize the ArUco detection results
    rqt_image = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        arguments=["/aruco_single/result"],
    )
    
    # Bridge for the SetEntityPose Service
    bridge_service = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        arguments=["/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
    )
    
    # Collect all declared arguments, nodes, and event handlers
    return LaunchDescription(
        declared_arguments
        + [ 
        gz_ign,
        robot_state_publisher_node,
        spawn_entity_node, 
        clock_bridge,
        bridge_camera,
        rqt_image,
        bridge_service,
        spawn_jsb_handler,
        spawn_controller_handler,
        ]
    )
