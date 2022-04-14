The recorded bag does *not* include an `/initialpose`.
This makes it robust and it can be run on a launch file as follows:
1. Start the `turtle_nav_filter` launch file using
```bash
ros2 launch turtle_nav_cpp turtle_nav_filter.launch.py
```
2. Set the initial pose on RVIZ. This the dead-reckoning estimation
3. Playback the recorded bag
```bash
ros2 bag play <bag-name>
```
Note that if the initial pose is not set, then only the true pose will be updated, but not the estimated pose (there may not even be an estimated node/ellipse).
