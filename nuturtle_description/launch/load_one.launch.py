from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation clock [true/false]'),
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            description='Whether to use joint_state_publisher [true/false]'),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            description='Whether to launch rvis2 [true/false]'),
        DeclareLaunchArgument(
            name='base_color',
            default_value='purple',
            choices=['red', 'blue', 'green', 'purple'],
            description='Select color for base [red,blue,green,purple]'),
        DeclareLaunchArgument(
            'r_frame',
            default_value='',
            description='Name of the fixed frame in RVIZ.'
        ),
        Node(
            package='robot_state_publisher',
            namespace=PathJoinSubstitution(
                [LaunchConfiguration('base_color'), '']
                                          ),
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_prefix':
                PathJoinSubstitution([LaunchConfiguration('base_color'), '']),
                "robot_description":
                Command(['xacro', ' ',
                        PathJoinSubstitution([
                            FindPackageShare(package='nuturtle_description'),
                            'urdf', 'xacro', 'turtlebot3_burger.urdf.xacro']),
                        ' base_color:=', LaunchConfiguration('base_color')])}
                        ],
            arguments=[PathJoinSubstitution([
                FindPackageShare(package='nuturtle_description'),
                'urdf', 'xacro', 'turtlebot3_burger.urdf.xacro'])]
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_jsp')),
            package='joint_state_publisher',
            namespace=PathJoinSubstitution([
                LaunchConfiguration('base_color'), '']),
            executable='joint_state_publisher',
            name='joint_state_publisher'),
        DeclareLaunchArgument(
            'rviz_config_name',
            default_value=['basic_',
                           LaunchConfiguration('base_color'), '.rviz'],
            description="Set path to configuration based on base_color"),
        SetLaunchConfiguration('rviz_config_file',
                               PathJoinSubstitution([
                                FindPackageShare(
                                    package='nuturtle_description'),
                                'config',
                                LaunchConfiguration('rviz_config_name')])),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            namespace=PathJoinSubstitution([
                LaunchConfiguration('base_color'), '']),
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',
                       PathJoinSubstitution([
                        FindPackageShare(package='nuturtle_description'),
                        'rviz', LaunchConfiguration('rviz_config_file')]
                                            ),
                       LaunchConfiguration('r_frame')],
            on_exit=Shutdown()
        ),
    ])
