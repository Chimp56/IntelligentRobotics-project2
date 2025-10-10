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
source devel/setup.bash
```

```bash
chmod +x ~/catkin_ws/src/project2/scripts/*.py
```

To start mapping the environment using SLAM, run:

```bash
roslaunch project2 mapping.launch
```


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
chmod +x scripts/*.py
roslaunch project2 mapping.launch

```