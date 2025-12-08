# TurtleBot3 Hand Gesture Recognition System (v1.0)

**Status:** Completed (Semester 1 Deliverable)  
**Developer:** Tytus Felbor
**Tech Stack:** ROS Noetic, MediaPipe, OpenCV, Python, TurtleBot3 (Raspberry Pi 4)

---

## 📖 Project Overview
This project implements a distributed control system that allows a user to drive a TurtleBot3 robot using hand gestures. The system uses a laptop (Ubuntu VM) as the "Brain" to process video and detect gestures, and the Raspberry Pi as the "Body" to execute motor commands.

<img width="1342" height="1024" alt="forward_tb3" src="https://github.com/user-attachments/assets/e321f4cb-2d22-442a-9720-85a99278f174" />
*Figure 1: Real-time hand tracking and gesture classification of closed palm running on the workstation.*

<img width="1342" height="1024" alt="right_tb3" src="https://github.com/user-attachments/assets/ac081369-1286-43df-844c-3ad7428bece9" />
*Figure 2: Real-time hand tracking and gesture classification of closed palm & thumb poiting right palm running on the workstation.*


<img width="1342" height="1024" alt="left_tb3" src="https://github.com/user-attachments/assets/a481bbed-79a4-4073-8eba-cbb5638f4c4c" />
*Figure 3: Real-time hand tracking and gesture classification of closed palm & thumb pointing left running on the workstation.*

### 🎥 Demo
https://github.com/user-attachments/assets/demo

---

## 🤖 System Architecture
The system operates on a distributed ROS network:

1.  **Gesture Node (Ubuntu VM):** Uses MediaPipe to detect hand landmarks. Maps poses (Open Palm, Fist, Thumb Direction) to ROS commands.
2.  **Communication Layer:** Publishes string commands to `/gesture_command` topic over WiFi.
3.  **Motion Node (Raspberry Pi):** Subscribes to commands and converts them into velocity inputs (`/cmd_vel`) for the motors. Includes a 3-second safety auto-stop.

| Gesture | Command | Robot Action |
| :--- | :--- | :--- |
| **Fist** | `GO` | Move Forward (0.2 m/s) |
| **Open Palm** | `STOP` | Stop immediately |
| **Thumb Left** | `LEFT` | Rotate Left (0.5 rad/s) |
| **Thumb Right** | `RIGHT` | Rotate Right (-0.5 rad/s) |

---

## 🛠️ Installation & Setup

### Prerequisites
* **Workstation:** Ubuntu 20.04 (VM or Native) with Webcam
* **Robot:** TurtleBot3 (Burger/Waffle) with Raspberry Pi 4
* **Network:** Both devices must be on the same WiFi network.

### 1. Setup Workstation (The "Brain")
```bash
# Clone the repository
git clone [https://github.com/](https://github.com/)TFelbor/turtlebot3-gesture-control.git
cd turtlebot3-gesture-control
```
# Install Python dependencies
```bash
pip3 install mediapipe opencv-python
```
# Build the workspace
```bash
catkin_make
source devel/setup.bash
```
### 2. Setup Raspberry Pi (The "Body")
SSH into the robot and clone the same repo into your catkin workspace src folder.
```bash
# On Raspberry Pi
cd ~/catkin_ws/src
git clone [https://github.com/](https://github.com/)TFelbor/turtlebot3-gesture-control.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
## 🚀 Usage

### Step 1: Bring up the Robot (On Pi)
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Step 2: Start Motion Control (On Pi)
```bash
rosrun motion_control motion_control_node.py
```

### Step 3: Start Gesture Recognition (On Workstation)
```bash
# Ensure ROS Master points to the Pi's IP
export ROS_MASTER_URI=http://[PI_IP]:11311
rosrun gesture_recognition gesture_recognition_node.py
```
