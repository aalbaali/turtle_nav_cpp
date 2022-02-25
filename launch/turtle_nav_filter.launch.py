#
# @author Amro Al-Baali (albaalia@live.com)
# @name turtle_nav_filter.launch.py
# @brief Kalman filter simulator
# 2022

from launch import LaunchDescription
from launch_ros.actions import Node

# True turtle topic name
TRUE_TURTLE_TOPIC_NAME = 'true_turtle'

# Estimated turtle topic name
EST_TURTLE_TOPIC_NAME = 'est_turtle'


def generate_launch_description():
    return LaunchDescription([
        # Turtlesim simulator
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            remappings=[
                ('turtle1/cmd_vel', TRUE_TURTLE_TOPIC_NAME + '/cmd_vel'),
                ('turtle1/pose', TRUE_TURTLE_TOPIC_NAME + '/pose'),
                ('turtle1/color_sensor', TRUE_TURTLE_TOPIC_NAME + '/color_sensor')
            ]
        ),

        # Estimator broadcaster
        Node(
            package='turtle_nav_cpp',
            executable='turtle_est_broadcaster',
            parameters=[
                {'target_name': 'est_turtle'},
                {'target_frame': 'est_turtle'},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'pose_subscribe_topic': TRUE_TURTLE_TOPIC_NAME + '/pose'}
            ]
        ),
    ])
