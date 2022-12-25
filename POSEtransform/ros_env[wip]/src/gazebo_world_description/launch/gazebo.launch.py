import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro
import yaml


def generate_launch_description():
    # Launch Arguments
    
    
    
    ###############################################################
    gazebo_world_description_path = os.path.join(
        get_package_share_directory('gazebo_world_description'))

    world_file = os.path.join(gazebo_world_description_path,
                              'world',
                              'world.sdf')
    multibot_config_file = os.path.join(gazebo_world_description_path,
                              'config',
                              'multibot_def.yaml')
    ###############################################################

    ###############################################################
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        model_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + gazebo_world_description_path + '/models'
    else:
        model_path =  gazebo_world_description_path + '/models'
    ############################################################### 
    bridge_config_file_param = 'config_file:=' + os.path.join(get_package_share_directory('gazebo_world_description'),'config','gz_bridge.yaml')
    
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p' ,bridge_config_file_param]
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ################################################################  
    return LaunchDescription([
        
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=model_path),
        
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 3 ' + world_file])]),
        node_ros_gz_bridge,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
