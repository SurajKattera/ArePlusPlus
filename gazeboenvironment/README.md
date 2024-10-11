# ArePlusPlus
RS1 Smart Factory Project w/ Autonomous Robots conducting SLAM and Path Planning

# Environment 
## How to add 
- Download .zip file 
- Extract 
- Put these files (Apriltag, shelf, table, person_standing, cardboard_box) into the following directory:
```
/home/student/.gazebo
```
- Put `warehousever3.0.world` in  `/home/student/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo`
- Put `warehouse_env.launch.py`  in `/home/student/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo`
- Then build:
```
cd ros2_ws
colcon build 
source install/setup.bash
```
