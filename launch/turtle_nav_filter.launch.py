#
# @author Amro Al-Baali (albaalia@live.com)
# @name turtle_nav_filter.launch.py
# @brief Kalman filter simulator
# 2022

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# True turtle topic name
TRUE_TURTLE_TOPIC_NAME = "true_turtle"

# Estimated turtle topic name
EST_TURTLE_TOPIC_NAME = "est_turtle"


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    # Load config files
    config_params = os.path.join(
        get_package_share_directory("turtle_nav_cpp"), "launch", "config", "turtle_nav_filter.yaml"
    )

    # Turtlesim simulator
    ld.add_action(
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim",
            remappings=[
                ("turtle1/cmd_vel", TRUE_TURTLE_TOPIC_NAME + "/cmd_vel"),
                ("turtle1/pose", TRUE_TURTLE_TOPIC_NAME + "/pose"),
                ("turtle1/color_sensor", TRUE_TURTLE_TOPIC_NAME + "/color_sensor"),
            ],
        )
    )

    # Estimate broadcaster
    ld.add_action(
        Node(
            package="turtle_nav_cpp",
            executable="turtle_est_broadcaster",
            parameters=[config_params],
        )
    )

    # Steering wheel encoder node
    ld.add_action(
        Node(
            package="turtle_nav_cpp",
            executable="steering_wheel_encoder",
            parameters=[config_params],
        )
    )

    # Position sensor node
    ld.add_action(
        Node(package="turtle_nav_cpp", executable="position_sensor", parameters=[config_params])
    )

    return ld
