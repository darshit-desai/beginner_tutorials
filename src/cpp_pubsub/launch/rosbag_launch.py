from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    args_Frequency = DeclareLaunchArgument('freq', default_value=TextSubstitution(text="600"))
    ros_bag_arg = DeclareLaunchArgument('rosbag_record', default_value=TextSubstitution(text="True"), choices=['True', 'False'], description="Bool for switching ROS bag recording on/off")

    publisher = Node(
        package='cpp_pubsub',
        executable='talker',
        parameters=[
            {"freq": LaunchConfiguration('freq')}
        ],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    subscriber = Node(
        package='cpp_pubsub',
        executable='listener'
    )

    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rosbag_record')),
        cmd=['ros2', 'bag', 'record', '-a'],
        shell=True
    )

    return LaunchDescription([
        args_Frequency,
        ros_bag_arg,
        publisher,
        subscriber,
        recorder
    ])
