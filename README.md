# Turtlebot Navigation

This [repository](https://github.com/yingruijie/turtlebot_nav.git) is for Question 1: Turtlebot Navigation, DASE 7503 Individual Assignment.


## Environment Install
Install ROS-Jazzy following [link](https://docs.ros.org/en/jazzy/Installation.html).
Then install the required packages.
```sh
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2 ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-bringup
```

## Run Program
```sh
# Terminal 1
mkdir -p dase7503_ws/src
cd dase7503_ws/src
git clone https://github.com/yingruijie/turtlebot_nav.git
cd ../..
colcon build
source install/setup.bash
ros2 run turtlebot_nav turtlebot_nav
# Terminal 2
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# Terminal 3
ros2 launch slam_toolbox online_async_launch.py
```



