from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory 

# change it according to agv
namespace_string = "agv1"

def generate_launch_description():
    
    
    camera_info_path = os.path.join(get_package_share_directory("agv_hardware"), 'config', 'camera_info.yaml')
    print(camera_info_path)
    container = ComposableNodeContainer(
        name='camera_container',
        namespace=namespace_string,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_cam',
                plugin='usb_cam::UsbCamNode',  # You'll need to verify this plugin name
                name=f'{namespace_string}_usb_cam_node',
                parameters=[
                    {'camera_name': 'default_cam'},
                    {'frame_id': f"{namespace_string}/camera_optical_link"},
                    {'video_device': "/dev/video0"},  # Change to scan1 to match scan_merger expectations
                    {'camera_info_url': f"file://{camera_info_path}"},
                    {'framerate': 15.0},
                    {'image_height': 480},
                    {'image_width': 640}
                ],
                remappings=[('image_raw', f'/{namespace_string}/image_raw'),
                            ('camera_info', f'/{namespace_string}/camera_info'),
                            ('image_raw/compressed', f'/{namespace_string}/image_raw/compressed'),
                            ('image_raw/compressedDepth', f'/{namespace_string}/image_raw/compressedDepth'),
                            ('image_raw/theora', f'/{namespace_string}/image_raw/theora'),
                            ('image_raw/zstd', f'/{namespace_string}/image_raw/zstd')
                        ]
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',  # You'll need to verify this plugin name
                name=f'{namespace_string}_apriltag_node', 
                remappings=[('image_rect', f'/{namespace_string}/image_raw'),
                            ('detections', f'/{namespace_string}/detections')
                        ]
            ),
            ComposableNode(
                package='agv_hardware',  # Your package name
                plugin='agv_hardware::TagTransformNode',   # Your class name
                name=f'{namespace_string}_tag_transform_node',
                parameters=[{'namespace': namespace_string}]
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