#
# @author Amro Al-Baali (albaalia@live.com)
# @name teleop_nav.launch.py
# @brief Launch a teleop node with remapped topic names
# 2022

from launch import LaunchDescription
from launch_ros.actions import Node

# True turtle topic name
TRUE_TURTLE_TOPIC_NAME = 'true_turtle'

# Estimated turtle topic name
EST_TURTLE_TOPIC_NAME = 'est_turtle'


def generate_launch_description():
    return LaunchDescription([
        # Teleop keys
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            remappings=[
                ('turtle1/cmd_vel', TRUE_TURTLE_TOPIC_NAME + '/cmd_vel')
            ]
        )
    ])
