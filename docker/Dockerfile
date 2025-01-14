ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#
# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
&& apt-get update \
&& apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN pip3 install setuptools
ENV SHELL /bin/bash

# Copy workspace
# TODO: Could also mount workspace instead of copying
# COPY --chown=$USERNAME:$USER_GID ./src/ src/
RUN mkdir -p /ros_templates/src
RUN chown -R $USERNAME /ros_templates
WORKDIR /ros_templates/

# Installing dependencies
RUN apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera
RUN apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros* \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    xterm \
    git \
    wget \
    python3-pip \
    terminator

# Try to fix sound
RUN mkdir /etc/modprobe.d
RUN echo "options snd_hda_intel index=1" > /etc/modprobe.d/default.conf

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME

COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]