# Project 2 - Hybrid Deliberative/Reactive Robotics

Navigate a set of points.

## Requirements
- Python2
- Ros melodic
- Gazebo
- Rviz

## Instructions

Clone the repository into your catkin workspace src:

```bashcd ~/catkin_ws/src
git clone https://github.com/Chimp56/IntelligentRobotics-project2 project2
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

```bash
chmod +x ~/catkin_ws/src/project2/scripts/*.py
```

To start mapping the environment using SLAM, run:

```bash
roslaunch project2 mapping.launch
```

To modify starting position
```bash
# Example: Start at custom position in feet and degrees (e.g., x=5.0 feet, y=8.0 feet, yaw=180 degrees)
roslaunch project2 mapping.launch x_feet:=5.0 y_feet:=8.0 yaw_deg:=180.0

# Default values: x_feet=1.0, y_feet=1.0, yaw_deg=90.0
roslaunch project2 mapping.launch
```

**Coordinate System:**
- +X axis: Right/East
- +Y axis: Forward/North  
- 0° heading: Pointing along +X (right)
- 90° heading: Pointing along +Y (forward)



To control turtlebot using keyboard, open a new terminal and run:
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

## Help



### keyboard movement

https://wiki.ros.org/cmd_vel_mux

### Vincent quick start up

```
git stash
git pull
chmod +x ~/catkin_ws/src/project2/scripts/*.py
roslaunch project2 mapping.launch


```