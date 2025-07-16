from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# change it according to agv
namespace_string = "agv1"

def generate_launch_description():
    
    
    # lidar_right_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     name='lidar_right_node',
    #     parameters=[
    #         {'serial_port':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_080e9b4db0808f48b24693469eb6cbf6-if00-port0'},
    #         {'frame_id':f"{namespace_string}/lidar_right_link"},
    #         {'topic_name':f"{namespace_string}/scanR"},
    #         {'angle_compensate':True},
    #         {'scan_mode':'Standard'},
    #         {'serial_baudrate':256000}
    #     ]
    # )

    # lidar_left_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     name='lidar_left_node',
    #     parameters=[
    #         {'serial_port':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_8f93963a0a60974ca252da49ac34b133-if00-port0'},
    #         {'frame_id':f"{namespace_string}/lidar_left_link"},
    #         {'topic_name':f"{namespace_string}/scanL"},
    #         {'angle_compensate':True},
    #         {'scan_mode':'Standard'},
    #         {'serial_baudrate':256000}
    #     ]
    # )

    # scan_merger_node = Node(
    #         package='agv_hardware',
    #         executable='scan_merger_v2',
    #         name='scan_merger_v2',
    #         namespace=namespace_string,
    #         output='screen'
    #     )

    container = ComposableNodeContainer(
        name='lidar_container',
        namespace=namespace_string,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rplidar_ros',
                plugin='rplidar_ros::rplidar_node',  # You'll need to verify this plugin name
                name='lidar_right_node',
                parameters=[
                    {'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_080e9b4db0808f48b24693469eb6cbf6-if00-port0'},
                    {'frame_id': f"{namespace_string}/lidar_right_link"},
                    {'topic_name': f"{namespace_string}/scanR"},  # Change to scan1 to match scan_merger expectations
                    {'angle_compensate': True},
                    {'scan_mode': 'Standard'},
                    {'serial_baudrate': 256000}
                ]
            ),
            ComposableNode(
                package='rplidar_ros',
                plugin='rplidar_ros::rplidar_node',  # You'll need to verify this plugin name
                name='lidar_left_node', 
                parameters=[
                    {'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_8f93963a0a60974ca252da49ac34b133-if00-port0'},
                    {'frame_id': f"{namespace_string}/lidar_left_link"},
                    {'topic_name': f"{namespace_string}/scanL"},  # Change to scan2 to match scan_merger expectations
                    {'angle_compensate': True},
                    {'scan_mode': 'Standard'},
                    {'serial_baudrate': 256000}
                ]
            ),
            ComposableNode(
                package='agv_hardware',  # Your package name
                plugin='ScanMergerV2',   # Your class name
                name='scan_merger_node',
                namespace=namespace_string
                # No additional parameters needed as it reads from namespace
            )
        ],
        output='screen'
    )
    

    

    return LaunchDescription([
        # lidar_right_node,
        # lidar_left_node,
        # scan_merger_node
        container
    ])