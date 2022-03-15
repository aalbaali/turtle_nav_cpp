FROM althack/ros2:foxy-dev

ENV _colcon_cd_root=/workspace

ARG WORKSPACE

# Copy the list of packages into the container
COPY packages.txt /scripts/

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
    # Install list of packages
    && xargs apt-get -y install --no-install-recommends < /scripts/packages.txt \
    && pip3 install --no-input gcovr \
    #
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

CMD ["/bin/bash"]
