#!/bin/bash

# Install dependencies
apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -y

# Build packages
colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' -Wall -Wextra -Wpedantic
