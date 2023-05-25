FROM ros:melodic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS, Gazebo, RViz, and other required packages
# Install required packages and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-velodyne-description \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    net-tools \
    vim \
    tmux \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Set up a catkin workspace
RUN mkdir -p /home/rosuser/catkin_ws/src

# Set the working directory
WORKDIR /home/rosuser/catkin_ws/src

# Clone the USV simulator repository
# Replace "your_usv_simulator_repository.git" with the actual repository URL
RUN git clone https://github.com/Cleanseasolutions/usv_simulator.git

# Change to the workspace root
WORKDIR /home/rosuser/catkin_ws/src/usv_simulator
RUN git pull

# Set upstream via ssh
RUN git remote add org-fork git@github.com:Cleanseasolutions/usv_simulator.git


# Change to the workspace root
WORKDIR /home/rosuser/catkin_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make --only-pkg-with-deps usv_msgs"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Source the setup.bash file
RUN echo "source /home/rosuser/catkin_ws/devel/setup.bash" >> /home/rosuser/.bashrc

# Set up X11 forwarding
RUN apt-get update && apt-get install -y --no-install-recommends x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Set up user and permissions for X11
RUN groupadd -g 1000 rosuser \
    && useradd -u 1000 -g 1000 -m -s /bin/bash rosuser \
    && echo "rosuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rosuser \
    && chmod 0440 /etc/sudoers.d/rosuser

# Change the ownership of the entire /home/rosuser directory and its subdirectories
RUN chown -R rosuser:rosuser /home/rosuser

# Set up the environment and enable colors in the terminal
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/rosuser/.bashrc \
    && echo 'force_color_prompt=yes' >> /home/rosuser/.bashrc \
    && echo 'export TERM=xterm-256color' >> /home/rosuser/.bashrc

# Switch to the new user
USER rosuser
WORKDIR /home/rosuser
RUN git config --global user.email "mbheir@gmail.com"
RUN git config --global user.name "Martin Heir"
RUN /bin/bash -c "source /home/rosuser/catkin_ws/devel/setup.bash"

# Expose ports for ROS
EXPOSE 11311
EXPOSE 9090

# Set entrypoint
CMD ["/bin/bash"]
