FROM ros:foxy

RUN sudo apt update 
RUN sudo apt install -y git 
RUN sudo apt install -y wget 
RUN sudo apt install -y unzip 
RUN sudo apt install -y libglib2.0-dev 
RUN sudo apt install -y iputils-ping 
RUN sudo apt install -y libboost-all-dev
RUN sudo apt install -y nano 
RUN sudo apt install -y ros-foxy-joy  
# RUN sudo apt install -y ros-foxy-velodyne 
# RUN sudo apt install -y ros-foxy-teleop-twist-keyboard 

# Install our source code (A1 ROS2 drivers)
RUN mkdir -p /home/mistlab/ros2_ws/src
WORKDIR /home/mistlab/ros2_ws/src
RUN git clone https://github.com/roman2veces/ros2_unitree_legged_msgs && \
    git clone https://github.com/roman2veces/unitree_ros2_to_real

# Install LCM library
WORKDIR /home/mistlab/
RUN wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip && \
    unzip v1.5.0.zip
RUN rm -rf v1.5.0.zip && mkdir lcm-1.5.0/build 
WORKDIR lcm-1.5.0/build 
RUN cmake .. && make && sudo make install && sudo ldconfig -v

# Install unitree_legged_sdk 3.2 version (fork) 
WORKDIR /home/mistlab/ros2_ws/src/unitree_ros2_to_real
RUN git clone https://github.com/roman2veces/unitree_legged_sdk 
RUN mkdir -p unitree_legged_sdk/build
WORKDIR unitree_legged_sdk/build 
RUN cmake .. && make

# Set environment variables
# 3_1 is for Aliengo robot, 3_2 is for A1 robot
ENV UNITREE_SDK_VERSION=3_2
ENV UNITREE_LEGGED_SDK_PATH=/home/mistlab/ros2_ws/src/unitree_ros2_to_real/unitree_legged_sdk
# ATTENTION: change this environment variable if you are running this docker image in a different architecture
# possible values: amd64, arm32, arm64 
ENV UNITREE_PLATFORM="amd64"

WORKDIR /home/mistlab/ros2_ws/
RUN . /opt/ros/foxy/setup.sh && colcon build

# Setup /root/.bashrc file 
RUN touch /root/.bashrc \
    && echo "source /opt/ros/foxy/setup.bash \nsource /home/mistlab/ros2_ws/install/setup.bash" >> /root/.bashrc
