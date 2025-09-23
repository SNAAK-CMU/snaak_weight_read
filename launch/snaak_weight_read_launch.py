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
            name='snaak_scale_bins_right',
            parameters=[
                {'node_name': 'snaak_scale_bins_right'},
                {'serial_port': '/dev/snaak_scale_bins_right'}
            ]
        ),
        Node(
            package='snaak_weight_read',
            executable='snaak_weight_service_node.py',
            name='snaak_scale_bins_left',
            parameters=[
                {'node_name': 'snaak_scale_bins_left'},
                {'serial_port': '/dev/snaak_scale_bins_left'}
            ]
        )
    ])