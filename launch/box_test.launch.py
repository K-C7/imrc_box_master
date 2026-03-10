from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    box_master_node = Node(
        package='imrc_box_master',
        executable='box_master_ransac',
        name='box_master',
        parameters=[{'debug': True}],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    angle_judge_node = Node(
        package='ros_angle_judgement',
        executable='angle_judgement',
        name='angle_judgement',
        arguments=['--ros-args', '--log-level', 'warn'],

    )

    return LaunchDescription([
        box_master_node,
        angle_judge_node
    ])

