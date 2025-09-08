import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory('path_planner'),
        'config')
    
    # graph_search_controller creates the graph that will be walked over
    graph_search_controller_node = Node(
        package='path_planner',
        executable='graph_search_controller_node',
        output='screen',
        parameters=[os.path.join(config_path, 'parameters.yaml'),
                    {'filename': os.path.join(config_path, 'bastide_map.mat'),
                     'resolution': 0.00765}])
    
    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(config_path, 'planner_conf.rviz')]
        )

    ld.add_action(graph_search_controller_node)
    ld.add_action(rviz_node)

    return ld