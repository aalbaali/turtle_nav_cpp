FROM althack/ros2:foxy-dev

ENV _colcon_cd_root=/workspace

ARG WORKSPACE

# Copy the list of packages into the container
COPY packages.txt /scripts/

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
    # Install list of packages
    && xargs apt-get -y install --no-install-recommends < /scripts/packages.txt \
    #
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
# RUN echo "$PWD"
# RUN echo "$(ls -lFa)"
# # Install dependencies
# RUN apt-get update \
#     && rosdep update && rosdep install --from-paths /workspace/src --ignore-src -y

# # Build package
# RUN colcon build --merge-install --symlink-install --cmake-args \
#     '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' \
#     -Wall -Wextra -Wpedantic


# # Install colcon test tools
# RUN git clone https://github.com/aalbaali/colcon_test_tools.git /Dev/colcon_test_tools \
#   && cd /Dev/colcon_test_tools \
#   && ./install.sh \
#   && source /usr/local/bin/colcon_tools.sh

# # Build tests
# RUN colcon test --merget-install

# CMD ["source /usr/local/bin/colcon_tools.sh && colcon_test"]

CMD ["/bin/bash"]
