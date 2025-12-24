from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动左手控制节点
        Node(
            package='hand_control_cpp', # 替换为你的包名
            executable='gripper_hand_controller',
            name='left_hand_controller', # 设置节点名，方便调试
            output='screen',
            parameters=[
                {'arm_side': 'left_arm'},
                {'gripper_sub_topic': 'gripper_cmd'},
                {'rate_hz': 20.0}
            ]
        ),
        
        # 启动右手控制节点
        Node(
            package='hand_control_cpp', # 替换为你的包名
            executable='gripper_hand_controller',
            name='right_hand_controller',
            output='screen',
            parameters=[
                {'arm_side': 'right_arm'},
                {'gripper_sub_topic': 'gripper_cmd'},
                {'rate_hz': 20.0}
            ]
        )
    ])