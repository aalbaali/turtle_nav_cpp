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

# Setting parameters
```bash
ros2 run <package-name> <node-name> --ros-args -p <param_name>:=<param_value>
```

## Starting teleop
Starting teleop using
```bash
ros2 run turtlesim turtle_teleop
```
publishes to `/turtle1/cmd_vel` by default.
However, in this package, velocities are published/subscribed from different topics such as `/turtle_true/cmd_vel`.
It's possible to publish teleop to a different topic by remapping the topics.
```bash
ros2 run turtlesim turtle_teleop --ros-args --remap /turtle1/cmd_vel:=/turtle_true/cmd_vel
```

# Running tests
To run the tests locally, run the `.devcontainer/run_all.sh` script
