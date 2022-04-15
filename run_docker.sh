#!/bin/bash

cd .ci

# Install dependencies
docker-compose run ros2-ws /bin/bash -c \
                   "apt-get update && rosdep update \
                    && rosdep install --from-paths src --ignore-src -y"
# Build packages
docker-compose run ros2-ws /bin/bash -c \
                    "colcon build --merge-install --symlink-install \
                   --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' \
                   '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' -Wall -Wextra -Wpedantic"

# Clone and install colcon-tools
docker-compose run ros2-ws /bin/bash -c \
  "if [[ ! -d colcon_test_tools ]]; then git clone https://github.com/aalbaali/colcon_test_tools.git colcon_test_tools; fi"


# Build tests
docker-compose run ros2-ws /bin/bash -c "colcon test --merge-install"

# Run tests
docker-compose run ros2-ws /bin/bash -c "source colcon_test_tools/colcon_tools.sh; colcon_test"

# Clean/remove
docker-compose run ros2-ws /bin/bash -c "colcon build --cmake-target clean --merge-install"

# Remove docker-compose
docker-compose down

cd -
