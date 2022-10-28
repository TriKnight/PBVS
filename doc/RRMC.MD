Run the code to start the robot
 ---------------Disable the robot Ethernet/IP Adapter ------------------------

``` 
In the PolyScope/Installation/Fieldbus/Ethernet-IP --> Disable
```
Start robot program

```
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.101 
```

Swicth to the velocity controller 

```
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
stop_controllers: ['scaled_pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.0"
```
