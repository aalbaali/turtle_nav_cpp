#
# @author Amro Al-Baali (albaalia@live.com)
# @name turtle_nav_filter.launch.py
# @brief Kalman filter simulator
# 2022

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='true'
        ),
    ])
