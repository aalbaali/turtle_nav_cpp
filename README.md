![Linting](https://github.com/aalbaali/turtle_nav_cpp/actions/workflows/lint.yml/badge.svg)
![Unit tests](https://github.com/aalbaali/turtle_nav_cpp/actions/workflows/tests.yml/badge.svg)

[![Codecov](https://codecov.io/gh/aalbaali/turtle_nav_cpp/branch/devel/graph/badge.svg?token=ENILSW0ES2)](https://codecov.io/gh/aalbaali/turtle_nav_cpp)

- [In this repo](#in-this-repo)
- [Nodes](#nodes)
  - [`turtle_est_broadcaster`](#turtle_est_broadcaster)
    - [Parameters](#parameters)
- [Running the dead-reckoning filter](#running-the-dead-reckoning-filter)
  - [Moving turtle using teleop](#moving-turtle-using-teleop)
- [Setting parameters](#setting-parameters)
- [Testing](#testing)
- [Pre-commits](#pre-commits)
- [Examples](#examples)

# In this repo
The turtlesim is used to implement a Kalman filter that estimates the turtle's position from noisy measurements.

# Nodes
## `turtle_est_broadcaster`
This node spawns a new turtle to the turtlesim and broadcasts the estimated pose.
Two `TF2` transforms are published.
Namely,
- `odom->est_pose`
- `map->odom`

### Parameters
The node takes multiple (optional) parameters
- `target_name`: Estimated turtle name
- `target_frame`: Estimated turtle TF2 frame name
- `odom_frame`: Odometry frame name
- `map_frame`: Map frame name

# Running the dead-reckoning filter
- Start the launch file by running
```bash
ros2 launch turtle_nav_cpp turtle_nav_filter.launch.py
```
- Start the dead-reckoning estimator by setting the pose:
  - From the launched RVIZ window, set a pose by clicking on the **2D Pose Estimate** button on the top of the screen.
  - Choose an appropriate pose estimate to kick-off the dead-reckoning estimator with (note: without this step, the estimator will not start)
- To move the robot around, run a [teleop session](#moving-turtle-using-teleop)
- A screenshot of RVIZ and turtlesim windows are presented in the [images below](#dead_reckoning_rviz)
## Moving turtle using teleop
Starting teleop using
```bash
ros2 run turtlesim turtle_teleop
```
publishes to `/turtle1/cmd_vel` by default.
However, in this package, velocities are published/subscribed from different topics such as `/turtle_true/cmd_vel`.
It's possible to publish teleop to a different topic by remapping the topics.
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/true_turtle/cmd_vel
```
To run the turtle in a circle, publish a twist message at a constant rate, which can be done [as follows](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html#:~:text=So%2C%20to%20get%20the%20turtle%20to%20keep%20moving%2C%20you%20can%20run%3A)
```bash
ros2 topic pub --rate 1 /true_turtle/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

RVIZ output             |  Turtlesim output
:-------------------------:|:-------------------------:
![dead_reckoning_rviz](images/dead_reckon_est_se2_covariance_RVIZ.png)  |  ![turtlesim_dead_reckon](images/turtlesim_dead_reckon.png)
The green polygon in the image above is the 99% uncertainty bound mapped to the *SE(2)* group, whereas the magenta ellipse is the 99% uncertainty bound in the *se(2)* *Lie algebra*.| Turtleisim view (there's an intentional offset between the two turtles).


# Setting parameters
```bash
ros2 run <package-name> <node-name> --ros-args -p <param_name>:=<param_value>
```

# Testing
The unit tests can be run using [colcon_test_tools](https://github.com/aalbaali/colcon_test_tools).

To run the tests locally, run the `.devcontainer/run_all.sh` script

# Pre-commits
To use:
```bash
pre-commit run -a
```
Or:
```bash
pre-commit install  # (runs every time you commit in git)
```
To update the `.pre-commit-config.yaml` file:
```bash
pre-commit autoupdate
```
Check [pre-commit](https://pre-commit.com/) for further details on `pre-commit`.

# Examples
The `examples` directory includes stand-alone examples that are not necessarily directly related to the package.
For example, it may have examples about propagating uncertainties.
To build the examples, pass `-DBUILD_EXAMPLES=ON` as a CMake argument.

The image below is the output of the *SE(2) dead-reckoning* example, where the "banana curve" is the 99% uncertainty bounds.
<p align="center">
  <img src="images/example_se2_dead_reckoning.png" />
</p>
