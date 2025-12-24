import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm_config = os.path.join(
        get_package_share_directory('rm_driver'),
        'config',
        'allrobot_config.yaml'
    )

    return LaunchDescription([

        Node(
            package='rm_driver',
            executable='rm_driver',
            name='left_rm_driver',
            namespace='left_arm',
            parameters=[arm_config],
            output='screen'
        ),

        Node(
            package='rm_driver',
            executable='rm_driver',
            name='right_rm_driver',
            namespace='right_arm',
            parameters=[arm_config],
            output='screen'
        ),
    ])
