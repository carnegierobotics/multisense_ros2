FROM osrf/ros:jazzy-desktop

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

ENV CRL_WS=/opt/crl
ENV ROS2_WS=$CRL_WS/ros_ws
RUN mkdir -p $ROS2_WS/src

WORKDIR $ROS2_WS

RUN apt-get update \
    && apt-get install -y \
        clangd \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 and beyond have a default `ubuntu` user.
# This is used to match the host user so files created in the container have host ownership.
RUN chown -R ubuntu:ubuntu $CRL_WS

# Grant passwordless sudo to the 'ubuntu' user.
# This permits the developer to perform operations as root, if needed (e.g. installing packages).
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Enable color prompt for the ubuntu and root users.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc /home/ubuntu/.bashrc

# Set ubuntu as the default user.
USER ubuntu
