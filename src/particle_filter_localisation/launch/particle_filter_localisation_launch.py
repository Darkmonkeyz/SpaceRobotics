import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Path common to the various world files
    marsyard_path = os.path.join(get_package_share_directory('particle_filter_localisation'),
            'world_data',
            'marsyard2022')

    # Stage simulator
    stage_node = Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{'one_tf_tree': False,
                        'enforce_prefixes': False,
                        'use_static_transformations': True,
                        'world_file': marsyard_path+'_map.world'}],
        )

    # map_server displays the map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': marsyard_path+'.yaml',
                     'frame_id': 'odom'}])
    
    
    # lifecycle_manager ensures map_server runs properly
    lifecycle_manager_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}])
    
    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('particle_filter_localisation'),
            'config',
            'visualisation.rviz')])

    # visual_terrain_sensor simulates a terrain sensor
    visual_terrain_sensor_node = Node(
        package='particle_filter_localisation',
        executable='visual_terrain_sensor',
        output='screen',
        parameters=[{'filename_class_map': marsyard_path+'_class.bmp',
                     'filename_class_colour_map': marsyard_path+'_class_coloured.bmp',
                     'filename_obstacles_map': marsyard_path+'_obstacles.bmp'},
                     marsyard_path+'_parameters.yaml'])

    # magnetometer simulates a magnetic field sensor
    magnetometer_node = Node(
        package='particle_filter_localisation',
        executable='mock_magnetometer',
        output='screen')
    
    # particle_filter_localisation is how the robot estimates where it is
    particle_filter_localisation_node = Node(
        package='particle_filter_localisation',
        executable='particle_filter_localisation',
        output='screen',
        parameters=[{'filename_class_map': marsyard_path+'_class.bmp',
                     'filename_class_colour_map': marsyard_path+'_class_coloured.bmp',
                     'use_terrain': True,
                     'use_laser': True,
                     'use_compass': False},
                     marsyard_path+'_parameters.yaml'])
    
    ld.add_action(stage_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    ld.add_action(visual_terrain_sensor_node)
    ld.add_action(magnetometer_node)
    ld.add_action(particle_filter_localisation_node)

    return ld