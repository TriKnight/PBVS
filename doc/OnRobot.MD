# ON Robot and ROS Driver installation
# 1. RG Grippers
This stack can be used to control OnRobot RG2 / RG6 and VG10 / VGC10. Only grippers configured with the Modbus / TCP protocol can be controlled using the stack. 
http://wiki.ros.org/onrobot
### Installing

```
git clone https://github.com/Osaka-University-Harada-Laboratory/onrobot
pip install pymodbus
```

### Run the RG Gripper Driver Node

The Gripper is driven by the node "OnRobotRGTcpNode.py" contained in the package "onrobot_rg_control". The IP address of the Gripper has to be provided as an argument.

For example, the driver for controlling an RG2 /RG6 Gripper with the IP address 192.168.1.1 is launched using the following command: 
```
roslaunch onrobot_rg_control bringup.launch ip:=192.168.1.1
```

### Testing run the RG Gripper Status Listener Node

In the package "onrobot_rg_control", there is also a node for listening to and interpreting the status of the Gripper. The driver publishes the status of the Gripper on "OnRobotRGInput". The status listener node can be run together with the driver node using the following same command: 
```
roslaunch onrobot_rg_control bringup.launch ip:=192.168.1.1
```


### Run the RG Gripper Simple Controller Node

The driver listens for messages on "OnRobotRGOutput". The messages are interpreted and commands are sent to the gripper accordingly. A simple controller node is provided which can be run (at another terminal) using: 
```
rosrun onrobot_rg_control OnRobotRGSimpleController.py
```
Result 
```
Simple OnRobot RG Controller
-----
Current command: rGFR = 400, rGWD = 1600, rCTR = 16
-----
Available commands

c: Close
o: Open
(0 - max width): Go to that position
i: Increase force
d: Decrease force

```

# 2. HEX OnRobot Torque Force Driver

```
https://github.com/jsbyysheng/onrobot_hex_ft_sensor
```
Running
```
roslaunch onrobot_hex_ft_sensor demo.launch 
```
Testing
```
rostopic echo /ethdaq_data
---
header: 
  seq: 6110
  stamp: 
    secs: 1648706325
    nsecs: 358567382
  frame_id: "ft_sensor_link"
wrench: 
  force: 
    x: 0.3464
    y: -2.5941
    z: -77.6874
  torque: 
    x: 0.86149
    y: 0.8467
    z: -0.03108
---

```