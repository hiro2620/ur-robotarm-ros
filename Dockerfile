FROM ros:humble

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive


# Install necessary packages
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur \
    ros-humble-librealsense2* \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*


# Create and set up the workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Clone the necessary repositories
RUN cd src && git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git -b humble --depth 1
# RUN cd src && git clone https://github.com/panagelak/rq_fts_ros2_driver.git --depth 1

# Install dependencies
RUN apt-get update -q && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Uncomment force_color_prompt in /root/.bashrc
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT []
CMD ["/bin/bash"]