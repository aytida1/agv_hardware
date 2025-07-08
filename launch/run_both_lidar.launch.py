from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# change it according to agv
namespace = "agv1"

def generate_launch_description():
    
    
    lidar_right_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_right_node',
        parameters=[
            {'serial_port':'/dev/ttyUSB1'},
            {'frame_id':f"{namespace}/lidar_right_link"},
            {'topic_name':f"{namespace}/scanR"},
            {'angle_compensate':'true'},
            {'scan_mode':'Standard'}
        ]
    )

    lidar_left_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_right_node',
        parameters=[
            {'serial_port':'/dev/ttyUSB2'},
            {'frame_id':f"{namespace}/lidar_left_link"},
            {'topic_name':f"{namespace}/scanL"},
            {'angle_compensate':'true'},
            {'scan_mode':'Standard'}
        ]
    )
    

    

    return LaunchDescription([
        lidar_right_node,
        lidar_left_node
    ])