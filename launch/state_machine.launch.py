from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_led',
            default_value='false',
            description='Whether to launch the LED light indicator node'
        ),
        DeclareLaunchArgument(
            'led_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the Circuit Playground Express LED ring'
        ),
        DeclareLaunchArgument(
            'led_msg_type',
            default_value='int',
            choices=['int', 'string'],
            description='Message type for the LED indicator (int or string)'
        ),

        Node(
            package='state_machine',
            executable='state_machine_node',
            name='simple_state_machine',
            output='screen',
        ),

        Node(
            package='state_machine',
            executable='led_light_indicator_node',
            name='led_light_indicator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_led')),
            arguments=[
                '--port', LaunchConfiguration('led_port'),
                '--msg-type', LaunchConfiguration('led_msg_type'),
            ],
        ),
    ])
