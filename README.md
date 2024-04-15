# ROS package for a pan tilt device made of two Dynamixel servos

https://github.com/AnthonyZJiang/dynamixel_pan_tilt/assets/8778250/a04f16a8-ec1d-4a47-b50e-7347ddd1794e

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
Command the pan tilt servos to move at the set velocity.

The servos are commanded in position mode where the gola positions are either the max or min positions depending on the direction of the velocity. The change of velocity is achieved by setting the profile velocity to the specified values.

### /cmd_inc
Command the pan tilt servos to move the specified position increment.

### /cmd_pos
*value from 0 to 1*

Command the pan tilt servos to move to the nomalised position (mapped from [min, max] to [0, 1]). 

## Services
### /home
Return to the home position specified by `dxl_pan_position_home` and `dxl_pan_position_home` parameters.

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

Address offset from the "indirect data" to its corresonding "indirect address". For example, for a Dynamixel XL330-M288, the address of `Indirect Address 1` is 168 and `Indrect Data` is 208, the offset is therefore 40.

### enable_present_velocity
*bool (default false)*

If true, `present velocity` will be added into indirect address/data for fast read. 

We use indirect read and write for faster communication with the servos. At least 16 addresses are required. Enabling `present velocity` requires 4 more address. You must check with the manual to confirm there are at least 20 indirect addresses before set this parameter to `true`.

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


