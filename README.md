# ROS package for the Ice Nine Mini Dynamixel Pan Tilt device

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