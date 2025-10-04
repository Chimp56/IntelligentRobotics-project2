# Project 2 - Hybrid Deliberative/Reactive Robotics

Uses subsumption architecture.

The behaviors your robot will carry out are as follows, ordered from highest priority to lowest:

1. Halt if collision(s) detected by bumper(s).
2. Accept keyboard movement commands from a human user.
3. Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
4. Avoid asymmetric obstacles within 1ft in front of the robot.
5. Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
6. Drive forward.

demo video

https://drive.google.com/file/d/1KNg_iiYCrzm1qfIFRs7pNybqrN1o7ksH/view?usp=sharing

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
roslaunch project2 reactive_mapping.launch
```

This will use gmapping and the reactive controller module found in /scripts/reactive_controller.py to navigate and map the environment.

To watch the mapping process, open a new terminal and run:

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

To control turtlebot using keyboard, open a new terminal and run:
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

## Help

https://wiki.ros.org/rospy/Overview/

https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

https://github.com/JdeRobot/base/pull/732/files#diff-02e337c3ee81c97584f3d34f9b17885bb26feecfa34ca1ddddd9d4fa2925256d

https://answers.ros.org/question/334143/

https://docs.ros.org/en/noetic/api/turtlebot3_msgs/html/msg/SensorState.html

https://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/BumperEvent.html

### escape obstacle

https://robotics.stackexchange.com/questions/74901/how-can-i-tell-the-distance-from-a-turtlebot-to-an-obstacle

https://www.mathworks.com/help/nav/ug/obstacle-avoidance-with-turtlebot-and-vfh.html

https://www.theconstruct.ai/read-laserscan-data/


### keyboard movement

https://wiki.ros.org/cmd_vel_mux

### Vincent reactive mapping debugger shortcut

```
git stash
git pull
chmod +x scripts/reactive_controller.py
roslaunch project2 reactive_mapping.launch

```