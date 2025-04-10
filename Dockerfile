ARG ros_codename=jazzy

FROM ros:${ros_codename}-ros-base
ARG ros_codename

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /opt/crl/ros_ws/src
WORKDIR /opt/crl/ros_ws

COPY . /opt/crl/ros_ws/src/multisense_ros

RUN . /opt/ros/${ros_codename}/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src -y --ignore-src \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN . /opt/ros/${ros_codename}/setup.bash \
    && colcon build \
    && colcon test \
    && rm -r /opt/crl/ros_ws/{install,build,log,src}
