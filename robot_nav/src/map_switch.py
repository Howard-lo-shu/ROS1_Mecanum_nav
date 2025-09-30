#!/usr/bin/python
import os
import yaml
import glob
import rospy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()
    directory = "/home/howard/cimc_ws/src/four_ws_amr_nav/maps"
    extension = ".yaml"  
    map_name=[]
    search_pattern = directory + '/*' + extension
    specific_files = glob.glob(search_pattern)
    
    if specific_files:
        rospy.loginfo(f"Found {len(specific_files)} {extension} files:")
        for file in specific_files:
            map_name.append(file) 
            #rospy.loginfo(file)
        
        map_number = map_name[0]
        print(map_number)
        #pub.publish(map_number)
    else:
        rospy.loginfo(f"No {extension} files found in directory.")

# map file
    map_file_path = os.path.join(map_number)

    map_server_cmd = Node(
        package='map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])


    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True, 
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld