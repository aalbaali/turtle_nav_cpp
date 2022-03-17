#
# @author Amro Al-Baali (albaalia@live.com)
# @name turtle_nav_filter.launch.py
# @brief Kalman filter simulator
# 2022

from launch import LaunchDescription
from launch_ros.actions import Node

# True turtle topic name
TRUE_TURTLE_TOPIC_NAME = "true_turtle"

# Estimated turtle topic name
EST_TURTLE_TOPIC_NAME = "est_turtle"

# Measurements namespace (i.e., any measurements should fall under it)
MEAS_TOPIC_NS = "meas"


# TODO(aalbaali): make this more module, similar to Hadabot's code
def generate_launch_description():
    # Create the launch description and populate
    # Functions that can be used with LaunchDescription:
    #   DeclareLaunchArgument
    ld = LaunchDescription()

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
            parameters=[
                {"target_name": "est_turtle"},  # Name of the spawned turtle
                {"target_frame": "est_turtle"},  # Estimated pose TF frame
                {"true_frame": "true_turtle"},  # True pose TF frame
                {"odom_frame": "odom"},  # Odometry TF frame
                {"map_frame": "map"},  # Map TF frame
                {"pose_subscribe_topic": TRUE_TURTLE_TOPIC_NAME + "/pose"},
                # Subscribe to this (temporarily subscribed to the true pose until the pose
                # estimate topic is read)
                # Request teleport service from this
                {"teleport_service": EST_TURTLE_TOPIC_NAME + "/teleport_absolute"},
            ],
        )
    )

    # Steering wheel encoder node
    ld.add_action(
        Node(
            package="turtle_nav_cpp",
            executable="steering_wheel_encoder",
            parameters=[
                {"true_meas_topic": TRUE_TURTLE_TOPIC_NAME + "/cmd_vel"},  # Subscribe to this
                {"noisy_meas_topic": MEAS_TOPIC_NS + "/wheel_encoder"},  # Publish to this
                {"linear_speed_noise_params": [0.0, 0.1]},  # [mean, std]
                {"angular_speed_noise_params": [0.0, 0.01]},  # [mean, std]
                {"meas_frame": "est_turtle"},  # Frame of measurement
            ],
        )
    )

    # Position sensor node
    ld.add_action(
        Node(
            package="turtle_nav_cpp",
            executable="position_sensor",
            parameters=[
                {"true_meas_topic": TRUE_TURTLE_TOPIC_NAME + "/pose"},  # Subscribe to this
                {"noisy_meas_topic": MEAS_TOPIC_NS + "/position"},  # Publish to this
                {"biases": [0.0, 0.0]},  # x, y
                {"covariance": [0.1, 0.0, 0.0, 0.1]},  # Row major order, just like ROS2
                {"meas_frame": "map"},  # Frame of measurement
            ],
        )
    )

    return ld
