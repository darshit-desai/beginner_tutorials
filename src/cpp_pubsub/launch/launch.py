from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    args_Frequency = DeclareLaunchArgument('freq', default_value=TextSubstitution(text="600"))
    return LaunchDescription([
        args_Frequency,
        Node(
            package='cpp_pubsub',
            executable='talker',
            parameters=[
                {"freq": LaunchConfiguration('freq')}
            ],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='cpp_pubsub',
            executable='listener'
        )
    ])
