# TurtleBot3 Hand Gesture Recognition System (v1.0)

This project implements a Remote Control by gesture for the TurtleBot3 robot. By leveraging computer vision and distributed robotics, I created a system that allows users to drive a mobile robot using intuitive hand gestures (e.g., a fist to drive forward, a thumb to turn left). 

---

## 📖 Executive Summary

**Project Status:** Completed (Semester 1 Deliverable)  
**Timeline:** September 2025 - December 2025

The TurtleBot3 Hand Gesture Recognition Control System is an R&D project designed to enable intuitive human-robot interaction through hand gesture recognition. This project combines computer vision (MediaPipe), real-time processing, and robotics (ROS + TurtleBot3) to create a direct teleoperation interface controlled entirely by hand gestures. The system will recognize five fundamental gesture commands (GO, STOP, LEFT, RIGHT) and translate them into robot movements, providing a foundation for future human-robot interaction research.

The system was built using **ROS Noetic** and **MediaPipe**, achieving real-time responsiveness with under 500ms latency.

[[Demo GIF]](https://github.com/user-attachments/assets/b3628aa3-530c-42d3-add7-b791bc518206)

<img width="1342" height="1024" alt="forward_tb3" src="https://github.com/user-attachments/assets/e321f4cb-2d22-442a-9720-85a99278f174" />

*Figure 1: Real-time hand tracking and gesture classification of closed palm running on the workstation.*

<img width="1342" height="1024" alt="right_tb3" src="https://github.com/user-attachments/assets/ac081369-1286-43df-844c-3ad7428bece9" />

*Figure 2: Real-time hand tracking and gesture classification of closed palm & thumb poiting right palm running on the workstation.*


<img width="1342" height="1024" alt="left_tb3" src="https://github.com/user-attachments/assets/a481bbed-79a4-4073-8eba-cbb5638f4c4c" />

*Figure 3: Real-time hand tracking and gesture classification of closed palm & thumb pointing left running on the workstation.*


---

## 🤖 System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                       UBUNTU VM (Development Machine)       │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  Gesture Recognition Node (Python + MediaPipe)          ││
│  │  - Capture video from built in camera                   ││
│  │  - Detect hand poses using MediaPipe                    ││
│  │  - Map poses to gesture commands (GO, STOP, etc.)       ││
│  │  - Publish commands to ROS topic: /gesture_command      ││
│  └─────────────────────────────────────────────────────────┘│
│              ↓ (ROS topic: /gesture_command)                │
│         Over Network (WiFi/SSH)                             │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                 RASPBERRY PI 4 (TurtleBot3)                 │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  Motion Control Node (Python + ROS)                     ││
│  │  - Subscribe to /gesture_command topic                  ││
│  │  - Translate commands to wheel velocities               ││
│  │  - Implement 3-second auto-STOP safety feature          ││
│  │  - Publish to /cmd_vel (motor control)                  ││
│  └─────────────────────────────────────────────────────────┘│
│              ↓ (ROS topic: /cmd_vel)                        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  TurtleBot3 Motor Drivers                               ││
│  │  - Actuate wheels based on velocity commands            ││
│  │  - Provide odometry feedback (optional)                 ││
│  └─────────────────────────────────────────────────────────┘│
│              ↓ (Physical)                                   │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  TurtleBot3 Chassis & Motors                            ││
│  │  - Execute movement commands                            ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### 🔄 ROS Node Communication Graph

```
Gesture Recognition Node          Motion Control Node
    (Ubuntu VM)                      (Raspberry Pi)
        │                                  │
        │ Publishes:                       │
        ├─→ /gesture_command ─────────────→ Subscribes
        │  (String: GO, STOP,              │
        │   LEFT, RIGHT, BACK-UP)          │
        │                                  │
        │                           Publishes:
        │                           │
        │                           ├─→ /cmd_vel
        │                              (Twist message)
        │                              
        │                           Subscribes:
        │                           │
        └───────────────────────────┴─ /gesture_timeout
                                       (auto-STOP signal)
```

### 🤙 Supported Gestures
| Gesture | Action | Description |
| :--- | :--- | :--- |
| **Fist** | **FORWARD** | Robot drives straight at 0.2 m/s |
| **Open Palm** | **STOP** | Robot halts immediately |
| **Thumb Left** | **TURN LEFT** | Robot rotates counter-clockwise |
| **Thumb Right** | **TURN RIGHT** | Robot rotates clockwise |

---

## 🛠️ Installation & Setup

### ⚠️ Hardware Setup & Prerequisites
**Important:** This repository assumes your TurtleBot3 is already physically assembled, the OpenCR board is configured, and the Raspberry Pi is networked.

If you are setting up the robot for the first time, you **must** complete the steps in the official **[TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)** first. This resource is helpful for:
* Assembling the hardware chassis.
* Flashing the Raspberry Pi with the correct ROS Noetic image.
* Configuring the OpenCR motor driver board.
* Setting up the basic network connection.

**System Requirements:**
* **Workstation:** Ubuntu 20.04 with Webcam (VM or Native)
* **Robot:** TurtleBot3 (Burger model) with Raspberry Pi 4
* **ROS Version:** Noetic
* **Network:** Both devices must be on the same WiFi network and be able to ping each other.

### 1. Workstation Setup (The "Brain")
```bash
# Clone the repository
git clone [https://github.com/](https://github.com/)TFelbor/turtlebot3-gesture-control.git

# Install dependencies
pip3 install mediapipe opencv-python

# Build workspace
cd turtlebot3-gesture-control
catkin_make
source devel/setup.bash
````

### 2. Robot Setup (The "Body")

*SSH into the Raspberry Pi and clone this repo into `~/catkin_ws/src`*

```bash
cd ~/catkin_ws/src
git clone [https://github.com/](https://github.com/)TFelbor/turtlebot3-gesture-control.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 How to Run

1.  **Start the Robot Hardware (Pi):**

    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```

2.  **Start Motion Listener (Pi):**

    ```bash
    rosrun motion_control motion_control_node.py
    ```

3.  **Start Vision System (Workstation):**

    ```bash
    # Ensure this points to your Robot's current IP
    export ROS_MASTER_URI=http://[ROBOT_IP]:11311
    rosrun turtlebot3_gesture gesture_recognition_node.py
    ```
