# How to run the AprilTag detection

Requirements:
```
sudo apt update
cd ~/ros2_ws/src
git clone https://github.com/AprilRobotics/apriltag_ros.git
git clone https://github.com/AprilRobotics/apriltag.git  # Core library
cd ~/ros2_ws
colcon build
source install/setup.bash
```
 
Install the apriltag-msgs
`sudo apt install ros-humble-apriltag-msgs`

In one terminal: 

`ros2 launch turtlebot3_gazebo warehouse_env.launch.py`

In 2nd terminal: 

`ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/image_raw   -r camera_info:=/camera/camera_info   -p family:=36h11   -p size:=0.5   -p max_hamming:=0`


The result will be published to the /detections topic.

