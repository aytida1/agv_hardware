from launch import LaunchDescription
from launch_ros.actions import Node



# change it according to agv
namespace_string = "agv1"

def generate_launch_description():

    vel_cmd_node = Node(
        package='agv_hardware',
        name='vel_cmd_node',
        executable='vel_cmd_to_serial.py',
        parameters=[
            {'namespace': namespace_string}
        ],
        output='screen'
    )

    control_servo_action_server_node = Node(
        package='agv_hardware',
        name='control_servo_action_server',
        executable='control_servo_action_server.py',
        parameters=[
            {'namespace': namespace_string}
        ],
        output='screen'
    )

    return LaunchDescription([
        vel_cmd_node,
        control_servo_action_server_node
    ])