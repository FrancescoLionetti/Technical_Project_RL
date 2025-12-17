import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_kdl_package')
    
    param_file_path = os.path.join(pkg_share, 'config', 'param.yaml')

    action_server_node = Node(
        package='ros2_kdl_package',
        executable='action_server_node',
        name='action_server_node',
        output='screen',
        # PASSAGGIO CHIAVE: Carica i parametri dal file YAML
        parameters=[param_file_path],
        
        remappings=[
            ('robot_state_publisher', '/iiwa/robot_state_publisher'),
            ('/joint_states', '/iiwa/joint_states'),
            ('/velocity_controller/commands', '/iiwa/velocity_controller/commands'),
            ('/effort_controller/commands', '/iiwa/effort_controller/commands'),
        ] 
    )

    return LaunchDescription([
        action_server_node
    ])
