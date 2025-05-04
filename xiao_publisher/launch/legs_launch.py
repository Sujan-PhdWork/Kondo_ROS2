from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xiao_publisher',
            executable='serial_pub',
            namespace='left_leg',
            name='left_leg',
            arguments=['/dev/ttyACM0']
        ),
        Node(
            package='xiao_publisher',
            executable='serial_pub',
            namespace='right_leg',
            name='right_leg',
            arguments=['/dev/ttyACM1']
        )
            ]
        )
    

