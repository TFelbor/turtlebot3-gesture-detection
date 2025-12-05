# TurtleBot 3 with Gesture Recognition
The repository showcases a turtlebot 3 Burger robot for gesture recognition, focusing on human-robot interaction and evaluating gesture recognition accuracy, developed over two semesters.

## Project Overview
This repository contains the implementation of a human-following robot using the TurtleBot 3 Burger for gesture recognition. The robot responds to simple hand gestures (e.g., "stop," "go," "turn") to control its behavior. The project emphasizes human-robot interaction (HRI) with a research focus on evaluating gesture recognition accuracy. Developed as a two-semester project (September 2025–May 2026), it includes simulation in Gazebo and real-world testing.

## Software

1. ROS 2 Jazzy: Framework for robot control and communication.
2. Python 3: Primary programming language for scripts and nodes.
3. OpenCV: For human detection and tracking (e.g., color-based or cascade classifiers).
4. MediaPipe: For gesture recognition (hand landmark detection).
5. Gazebo: Simulation environment for prototyping and testing.
6. RViz: Visualization tool for debugging and monitoring.
7. TensorFlow Lite: Lightweight ML for gesture recognition (used in later phases).
8. Jupyter: For data analysis and visualization during evaluation.
9. Matplotlib: For plotting performance metrics.
10. Git: Version control for collaborative development.

## Installation

1. Hardware Setup:
* Assemble the TurtleBot 3 Burger per the official e-Manual.
* Mount the camera on the front plate.
* Connect and test all hardware components (motors, IMU, encoders).


2. Software Setup:

* Install ROS 2 Jazzy on the Raspberry Pi 4 and workstations. Follow the ROS 2 installation guide.
* Install dependencies:sudo apt update
```bash
sudo apt install python3-opencv python3-pip
pip3 install mediapipe tensorflow
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs
```

* Set up ROS networking for SSH/remote control (ensure stable Ethernet or Wi-Fi).
* Install Gazebo:sudo apt install ros-jazzy-gazebo-ros-pkgs




3.  Repository Setup:

* Clone this repository: 
```bash
git clone https://github.com/TFelbor/Research-and-Development-Project.git
```

* Build the ROS workspace:
```bash
cd Research-and-Development-Project
colcon build
source install/setup.bash
```




## Usage

1. Teleoperation Test:
* Verify hardware functionality:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
* Ensure the camera streams correctly:
```bash
ros2 run image_transport republish --ros-args --remap /image_raw:=/camera/image_raw
```

2. Simulation:
* Launch Gazebo simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Test human-following in simulation before physical deployment.

3. Running the Prototype:
* Launch the human-following node (after prototyping phase):
```bash
ros2 run human_following follow_person
```
* Gesture recognition (post-January 2026):
```bash
ros2 run gesture_recognition gesture_control
```

## Contributing

* Use Git branches for features (e.g., feature/human-tracking, feature/gesture-recognition).
* Document code and experiments thoroughly.
* Log issues and metrics in the repository's Issues tab.

## Challenges and Notes

* Network Stability: Use Ethernet for reliable ROS communication.
* Lighting Conditions: Test vision algorithms in varied environments.
* Compute Limits: Offload heavy ML tasks to simulation or workstation if Raspberry Pi struggles.
* Backups: Regularly back up code and ROS bag data.

## Resources

* TurtleBot 3 e-Manual
* ROS 2 Jazzy Documentation
* MediaPipe Guide
* OpenCV Tutorials


