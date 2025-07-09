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
            {'serial_port':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_080e9b4db0808f48b24693469eb6cbf6-if00-port0'},
            {'frame_id':f"{namespace}/lidar_right_link"},
            {'topic_name':f"{namespace}/scanR"},
            {'angle_compensate':True},
            {'scan_mode':'Standard'},
            {'serial_baudrate':256000}
        ]
    )

    lidar_left_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_left_node',
        parameters=[
            {'serial_port':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_8f93963a0a60974ca252da49ac34b133-if00-port0'},
            {'frame_id':f"{namespace}/lidar_left_link"},
            {'topic_name':f"{namespace}/scanL"},
            {'angle_compensate':True},
            {'scan_mode':'Standard'},
            {'serial_baudrate':256000}
        ]
    )
    

    

    return LaunchDescription([
       lidar_right_node,
        lidar_left_node
    ])