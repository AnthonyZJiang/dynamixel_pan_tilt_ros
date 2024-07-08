# dynamixel_pan_tilt_ros
ROS package for a pan tilt device made of two Dynamixel servos.

An example of the hardware can be found in https://github.com/AnthonyZJiang/dynamixal-pan-tilt-camera-cad/

# Install
```
cd catkin_ws/src
git clone https://github.com/AnthonyZJiang/dynamixel_pan_tilt.git
cd ..
rosdep install --from-paths src -i -r -y
catkin_make
```

# Launch
```
roslaunch dynamixel_pan_tilt dym_pan_tilt.launch
```

# API

## Subscribed topics
### /cmd_vel
*Message type: [PanTiltCmd](dynamixel_pan_tilt_msgs/msg//PanTiltCmd.msg)*

Command the pan tilt servos to move at the set velocity.

Note: The servos are commanded in position mode where the goal positions are either the max or min positions (defined by `dxl_<pan/tilt>_position_<min/max>`) depending on the direction of the velocity. The velocity is varied by changing the profile velocity to the specified values.

### /cmd_inc
*Message type: [PanTiltCmd](dynamixel_pan_tilt_msgs/msg//PanTiltCmd.msg)*

Command the pan tilt servos to move by the specified position increment.

### /cmd_pos
*Message type: [PanTiltCmd](dynamixel_pan_tilt_msgs/msg//PanTiltCmd.msg)*

*value from 0 to 1*


Command the pan tilt servos to move to the desired position within the minimum and maximum range. is a parameter used to specify the desired position of a servo motor within its range.

- 0: Represents the minimum position of the servo motor, defined by `dxl_<pan/tilt>_position_min`
- 1: Represents the maximum position of the servo motor, defined by `dxl_<pan/tilt>_position_max`
- Values between 0 and 1 interpolate between the minimum and maximum positions linearly.

## Published topics
### /diagnostics
For each motor the following information is broadcasted at 10 hz:

- online status
- torque enabled status 
- hardware error presence, including over-load, over-heat, electronical shock, motor encoder error.
- goal position
- present position
- present velocity
- present load

### /status
*Message type: [JointStatus](dynamixel_pan_tilt_msgs/msg/JointStatus.msg)*

Publishes a message with updated information of each motor every 0.1 second if parameter `enable_present_velocity` is set to `true`. These information is the same as those published in `/diagnostics`, only useful if the user wants to retrieve the status without the need to filter through the diagnostics message.

## Services
### /home
Return to the home position specified by `dxl_<pan/tilt>_position_home` parameters.

### /soft_reboot
Execute a software reboot of **both** servos.

## Parameters
### device_name
*string (default /dev/ttyUSB0)*

Name of the U2D2 device.

### baud_rate
*int (default 1000000)*

Baud rate of the Dynamixel servo.

### addr_indirect_address_data_offset
*int (default 56)*

Address offset from the "indirect data" to its corresponding "indirect address". For example, for a Dynamixel XL330-M288, the address of `Indirect Address 1` is 168 and `Indirect Data` is 208, the offset is therefore 40.

### enable_present_velocity
*bool (default false)*

If true, `present velocity` will be added into indirect address/data for fast read. 

We use indirect read and write for faster communication with the servos. At least 16 addresses are required. Enabling `present velocity` requires 4 more addresses. You must check with the manual to confirm that there are at least 20 indirect addresses before setting this parameter to `true`.

### broadcast_joint_status
*bool (default false)*

If true, the joint status will be published to the topic `/status`, in addition to `/diagnostics`

### dxl_pan_id
*int (default 0)*

The id of the pan servo

### dxl_tilt_id
*int (default 1)*

The id of the tilt servo

### dxl_pan_profile_velocity_default
*int (default 100)*

Default profile velocity of the pan servo, used for position/incremental commands

### dxl_tilt_profile_velocity_default
*int (default 100)*

Default profile velocity of the tilt servo, used for position/incremental commands

### dxl_pan_profile_acceleration
*int (default 100)*

Default profile acceleration of the pan servo, used for position/incremental commands

### dxl_tilt_profile_acceleration
*int (default 100)*

Default profile acceleration of the tilt servo, used for position/incremental commands

### dxl_pan_position_max
*int (default 3300)*

Maximum position value of the pan servo

### dxl_tilt_position_max
*int (default 3300)*

Maximum position value of the tilt servo

### dxl_pan_position_min
*int (default 796)*

Minimum position value of the pan servo

### dxl_tilt_position_min
*int (default 796)*

Minimum position value of the tilt servo

### dxl_pan_position_home
*int (default 2048)*

Home position value of the pan servo

### dxl_tilt_position_home
*int (default 2048)*

Home position value of the tilt servo


