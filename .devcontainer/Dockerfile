# ros2_ws/.devcontainer/Dockerfile
FROM osrf/ros:humble-desktop-full

ARG USERNAME=r2humble
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=humble

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
RUN apt-get install -y git wget

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-argcomplete \
    python3-venv \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    iproute2 \
    vim \
    gdb \
    gdbserver

# Setup ROS environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc
RUN echo "alias vb='vi ~/.bashrc' " >> /home/$USERNAME/.bashrc
RUN echo "alias sb='source ~/.bashrc' " >> /home/$USERNAME/.bashrc

ENV SHELL /bin/bash

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Set the working directory
WORKDIR /workspace
USER $USERNAME
CMD ["/bin/bash"]