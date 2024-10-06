# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 Jazzy repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Set up Gazebo Harmonic repositories
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS 2 Jazzy, Gazebo Harmonic, and additional dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    gz-harmonic \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Create a new user to run GUI applications
RUN useradd -m pioneer-container && echo "pioneer-container:password" | chpasswd && adduser pioneer-container sudo
USER pioneer-container
WORKDIR /home/pioneer-container

# Create a ROS workspace
RUN mkdir -p ~/uav-llm-integration/src

# Copy the entire context (including your ROS project) into the container
COPY --chown=pioneer-container:pioneer-container src/ /home/pioneer-container/uav-llm-integration/src/

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd ~/uav-llm-integration && colcon build"

# Source the workspace in .bashrc
RUN echo "source ~/uav-llm-integration/install/setup.bash" >> ~/.bashrc

# Set environment variables
ENV GZ_SIM_RESOURCE_PATH=/home/pioneer-container/uav-llm-integration/install/uav_sim/share/

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]