import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    starter = Node(
        package='stepanova_anastasia_autorace_core',
        executable='starting',
        name='starter',
        parameters=[
        ]
    )
    
    foo_dir = get_package_share_directory('autorace_camera')
    included_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                foo_dir + '/launch/extrinsic_camera_calibration.launch.py'))
    
    return LaunchDescription([
        starter,
        included_launch,
    ])

