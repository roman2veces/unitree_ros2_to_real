# Introduction
This package can send control commands to real Unitree A1 robot from ROS2. 

At this moment, you can do only high-level control(namely control the walking direction and speed of robot). Low-level control (namely control all joints on robot) will be ready soon.

This version is suitable for unitree_legged_sdk v3.2.1.

# Dependencies:
(this should be all installed by the Dockerfile)

* [unitree_legged_sdk v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/releases?page=2)
* [ros2_unitree_legged_msgs (fork)](https://github.com/roman2veces/ros2_unitree_legged_msgs)
* [lcm](https://github.com/lcm-proj/lcm/archive/refs/tags/)

# Environment
Tested in Ubuntu 20.04 using ros2 foxy. For others operating systems see https://github.com/roman2veces/unitree_a1 

# Build
TODO

# Run the package
You need to open 3 terminals.

Terminal 1:
```
docker start -i a1_drivers
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch unitree_legged_real real.launch.py
```

Terminal 2:
```
docker exec -it a1_drivers bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 3:
```
docker exec -it a1_drivers bash
cd /home/mistlab/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run unitree_legged_real walk_example
```

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
