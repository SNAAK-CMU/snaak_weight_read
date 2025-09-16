from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snaak_weight_read',
            executable='snaak_weight_service_node.py',
            name='snaak_scale_assembly',
            parameters=[
                {'node_name': 'snaak_scale_assembly'},
                {'serial_port': '/dev/snaak_scale_assembly'}
            ]
        ),
        Node(
            package='snaak_weight_read',
            executable='snaak_weight_service_node.py',
            name='snaak_scale_bins',
            parameters=[
                {'node_name': 'snaak_scale_bins'},
                {'serial_port': '/dev/snaak_scale_bins_right'}
            ]
        )
    ])