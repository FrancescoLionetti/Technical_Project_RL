import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # definiamo la variabile params che contiene il percorso al file yaml contenente i parametri 
    params = PathJoinSubstitution(
        [FindPackageShare('ros2_kdl_package'), 'config', 'param.yaml']
    )
    
    #definiamo le variabili che possiamo passare da terminale quando lanciamo con launch
    #crea un argomento chiamato cmd_interface che come valore di default passa position
    cmd_interface = DeclareLaunchArgument(  
        'cmd_interface',
        default_value='position',
        description='Command interface (position, velocity, effort)'
    )
    
    #crea un argomento chiamato ctrl per specificare il tipo di controllore desiderato
    ctrl = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl', # Il vecchio controllore
        description='Controller type (velocity_ctrl, velocity_ctrl_null)'
    )

    #acquisizione dei valori degli argomenti
    command = LaunchConfiguration('cmd_interface')
    ctrl_type = LaunchConfiguration('ctrl')
    
    # Definizione del nodo
    ros2_kdl_node = Node(
        package='ros2_kdl_package',          #dove si trova l'eseguibile
        executable='ros2_kdl_node',          #nome eseguibile file cpp
        name='param_node',                   #nome che il nodo avrà all'interno del sistema
        output='both',                       #stampa i parametri sia a video che al robot
        parameters=[params,
        	{'cmd_interface': command},
            	{'ctrl': ctrl_type}
        ]                  #Passa il percorso del file di parametri
    )

    return LaunchDescription([
    	cmd_interface,
    	ctrl,
        ros2_kdl_node
    ])
