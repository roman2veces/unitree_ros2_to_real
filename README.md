# Introduction
This package can send control commands to real Unitree A1 robot from ROS2. 

At this moment, you can do only high-level control(namely control the walking direction and speed of robot). Low-level control (namely control all joints on robot) will be ready soon.

This version is suitable for unitree_legged_sdk v3.2.1.

# Build (using Docker)
Start by cloning this repo:
```
git clone https://github.com/roman2veces/unitree_ros2_to_real.git
```

if this is the first time you use this docker image, run the following commands: 

```
cd unitree_ros2_to_real
docker build -t <image name> .
docker run --name <container name> -v /dev/:/dev/ -it <image name>
source /opt/ros/foxy/setup.bash
colcon build
exit 
```

# Dependencies:
(this should be all installed by the Dockerfile)

* [unitree_legged_sdk v3.2 (fork)](https://github.com/roman2veces/unitree_legged_sdk)
* [ros2_unitree_legged_msgs (fork)](https://github.com/roman2veces/ros2_unitree_legged_msgs)
* [lcm](https://github.com/lcm-proj/lcm/archive/refs/tags/)

# Environment
Tested in Ubuntu 20.04 using ros2 foxy. For others operating systems see https://github.com/roman2veces/unitree_a1 

# Run the package
First, make sure that the A1 is on and standing up correctly. Then, connect your computer to 
the robot wifi and also connect the usb controller to your computer. 

Then, you need to open 2 terminals.

**Terminal 1:**
```
docker start -i <container name>
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch unitree_ros2_to_real joy_driver.launch.py
```

**Terminal 2:**
```
docker exec -it <container name> bash
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run unitree_ros2_to_real a1_twist_driver
```

# Run the package (NOT UP TO DATE)
First, make sure that the A1 is on and standing up correctly. Then, connect your computer to 
the robot wifi. 

Then, you need to open 3 terminals.

**Terminal 1:**
```
docker start -i <container name>
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch unitree_ros2_to_real joy_driver.launch.py
```

**Terminal 2:**
```
docker exec -it <container name> bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3:**
```
docker exec -it <container name> bash
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run unitree_ros2_to_real joy_driver
```

# Important points

- When building the docker image in the robot, the robot computer has a arm64 architecture. So, make sure you change the environment variable UNITREE_PLATFORM to arm64 in the dockerfile before building the image or changing the environment variable in your current docker container. When running the docker container in your computer you should use the value amd64.

# Setup the net connection (NOT UP TO DATE)
First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to be changed to your own.
