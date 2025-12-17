import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
 
def generate_launch_description():

    world_file = os.path.join(get_package_share_directory("airport_baggage_system"), "worlds", "airport_new.world")  
    models_path = os.path.join(get_package_share_directory('airport_baggage_system'), 'models')
    
    pkg_aruco_ros = get_package_share_directory("aruco_ros")
 	
    
    declared_arguments = [
        DeclareLaunchArgument("gz_args",
            default_value=["-r ", world_file],
            description="path to world file",
        )
    ]
    
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch",                    
                                      'gz_sim.launch.py'])]),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args"),
                              "publish_clock": "true",
                              }.items()
    )
 
    # Clock bridge to keep simulation time and ROS2 time synchronized
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    
    
    
    #--------------------------#
    #---------SETUP IIWA-------#
    #--------------------------#
    
    iiwa_xacro_file_name = "iiwa.config.xacro"
    iiwa_xacro = os.path.join(
        get_package_share_directory("iiwa_description"), "config", iiwa_xacro_file_name)
    
    
    iiwa_params = {"robot_description": Command(["xacro ", iiwa_xacro,
    						 " namespace:=iiwa/"])}
    
    
    #Robot State Publisher per IIWA
    iiwa_rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iiwa",
        output="screen",
        parameters=[
            iiwa_params,             # Passi il dizionario semplice
            {"use_sim_time": True}
        ],
    )

    iiwa_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/iiwa/robot_description", 
            "-name", "iiwa"
            ],   
    )
    
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/iiwa/controller_manager",
            "--controller-manager-timeout", "120",
        ],

    )
    
    iiwa_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/iiwa/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        
    )
    
    iiwa_jsb_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_spawn_node, 
            on_exit=[joint_state_broadcaster_node],
        )
    )
    
    iiwa_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[iiwa_controller_spawner],
        )
    )
    
    bridge_camera = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace="iiwa",
        arguments=[
            # Bridge Gazebo's '/camera' topic to a ROS 2 'sensor_msgs/msg/Image'
            "iiwa/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            # Bridge the '/camera_info' topic as well
            "iiwa/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "--ros-args",
            # Remap the ROS 2 topic '/camera' to '/stereo/left/image_rect_color'
            "-r",
            "iiwa/camera:=stereo/left/image_rect_color",
            # Remap the ROS 2 topic '/camera_info' to '/stereo/left/camera_info'
            "-r",
            "iiwa/camera_info:=stereo/left/camera_info",
            
        ],
        output="screen",
    )
    
    # Node to visualize the ArUco detection results
    rqt_image = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        #arguments=["/aruco_single/result"],
    )
    
    gripper_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gripper_bridge",
        arguments=[
            # Bridge per il Cubo ROSSO (Attach e Detach)
            "/iiwa/attach_red@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/iiwa/detach_red@std_msgs/msg/Empty@ignition.msgs.Empty",
            
            # Bridge per il Cubo VERDE (Attach e Detach)
            "/iiwa/attach_green@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/iiwa/detach_green@std_msgs/msg/Empty@ignition.msgs.Empty"
        ],
        output="screen",
    )
    
   
    #--------------------------#
    #---------SETUP FRA2MO-----#
    #--------------------------#
    
    fra2mo_xacro_file_name = "fra2mo.urdf.xacro"
    fra2mo_xacro = os.path.join(get_package_share_directory('ros2_fra2mo'), "urdf", fra2mo_xacro_file_name)

    fra2mo_models_path = os.path.join(get_package_share_directory('ros2_fra2mo'), 'models')
    
    fra2mo_robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', fra2mo_xacro]),value_type=str)}
    
    fra2mo_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="fra2mo",
        output="screen",
        parameters=[fra2mo_robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )
    
    fra2mo_jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace="fra2mo",
        parameters=[{"use_sim_time": True}]
    )
    
    position = [-8.35, 4.22, 0.0]

    # Define a Node to spawn the robot in the Gazebo simulation
    fra2mo_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/fra2mo/robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),]
    )
    
    fra2mo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace="fra2mo",
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   #'/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   #'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                   #'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                   '/fra2mo/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/fra2mo/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
                   
        remappings=[
        
        ('/cmd_vel', 'cmd_vel'),
        #('/model/fra2mo/odometry', 'odom'),
        #('/model/fra2mo/tf', 'tf'),
        ('/lidar', 'lidar'),
        ('/fra2mo/camera', 'camera'),
        ('/fra2mo/camera_info', 'camera_info')
    	],
     
        output='screen'
    )
    
    
    fra2mo_odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace="fra2mo",
        parameters=[{"use_sim_time": True}]
    )
    
    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        arguments=["/world/airport_baggage_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
    )
    
    fra2mo_cargo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_cargo_bridge',
        arguments=[
            # RED CUBE
            '/fra2mo/attach_red@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/fra2mo/detach_red@std_msgs/msg/Empty@ignition.msgs.Empty',
            
            # GREEN CUBE
            '/fra2mo/attach_green@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/fra2mo/detach_green@std_msgs/msg/Empty@ignition.msgs.Empty',
        ],
        output='screen'
    )
    
    ##-- ARUCO PER FRA2MO --##
    
    aruco_remaps_fra2mo = [
        ('/camera_info', '/fra2mo/camera_info'),
        ('/image', '/fra2mo/camera')
    ]

    
    aruco_fra2mo_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_tag',
        namespace='aruco_tag',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 1,         
            'reference_frame': '',
            'camera_frame': 'camera_link_optical',
            'marker_frame': 'marker_aruco_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        
        remappings=aruco_remaps_fra2mo,
        output='screen'
    )
    
    
    
    # Collect all declared arguments, nodes, and event handlers
    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + fra2mo_models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + declared_arguments + [
 
       gazebo_ignition,
        clock_bridge,
        
        #Nodi IIWA
        iiwa_rsp_node,
        iiwa_spawn_node,
        iiwa_jsb_handler,
        iiwa_controller_handler,
        bridge_camera,
        rqt_image,
        gripper_bridge,
        
        #Nodi Fra2mo
        fra2mo_rsp_node,
        fra2mo_jsp_node,
        fra2mo_spawn,
        fra2mo_bridge,
        fra2mo_odom_tf,
        fra2mo_cargo_bridge,
        aruco_fra2mo_node,
        
        set_pose_bridge	
        ]
    )
