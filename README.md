# 🐢 TurtleBot3 Hand Gesture Recognition System (v2.0)

This project implements remote teleoperation for the TurtleBot3 robot via real-time hand gestures. By leveraging computer vision and distributed robotics, users can drive a mobile robot using intuitive hand poses (e.g., a fist to drive forward, a thumb to turn left). 

As of the **v2.0 Update (`v2-cnn-pytorch` branch)**, this project features two selectable vision architectures: a heuristic model (MediaPipe) and a data-driven model (MobileNetV2 CNN), both communicating over a secure Tailscale VPN.

---

## 📖 Executive Summary

**Project Status:** Semester 2 Update (Dual-Architecture Framework)  
**Timeline:** Spring 2026  

The TurtleBot3 Hand Gesture Recognition Control System is an R&D project designed to enable intuitive human-robot interaction. The system recognizes fundamental gesture commands (GO, STOP, LEFT, RIGHT, WAIT) and translates them into `/cmd_vel` motor velocities. 

The v2.0 update focused on migrating from a purely logic-based landmark detection pipeline (MediaPipe) to a custom-trained Convolutional Neural Network (PyTorch/MobileNetV2). Additionally, a peer-to-peer Tailscale VPN was integrated to bypass local Wi-Fi router restrictions, ensuring seamless bidirectional ROS communication between the inference workstation and the Raspberry Pi.

[Drag and drop your Demo GIF here]

<img width="890" height="802" alt="camera_dashboard" src="https://github.com/user-attachments/assets/1fc46ddf-25bf-465c-a66d-a3147e578529" />

*Figure 1: The real-time camera dashboard highlighting PyTorch CNN gesture inference and the active Majority Vote Smoothing Filter.*

<img width="1036" height="828" alt="training_benchmark" src="https://github.com/user-attachments/assets/86d04c28-e8eb-4ed5-816a-74e1a109fe76" />

*Figure 2: Confusion matrix benchmarking the MobileNetV2 training results and accuracy on the validation set.*

---

## 🤖 System Architecture Diagram

```text
┌─────────────────────────────────────────────────────────────┐
│                       UBUNTU VM (The Brain)                 │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  Gesture Recognition Node (PyTorch CNN or MediaPipe)    ││
│  │  - Capture video from workstation camera                ││
│  │  - Run inference (MobileNetV2) or Landmark Detection    ││
│  │  - Apply 5-frame Majority Vote Smoothing Filter         ││
│  │  - Publish commands to ROS topic: /gesture_command      ││
│  └─────────────────────────────────────────────────────────┘│
│              ↓ (ROS topic: /gesture_command)                │
│          Over Tailscale VPN Tunnel (100.x.x.x)              │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                 RASPBERRY PI 4 (The Body)                   │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  Motion Control Node (Python + ROS)                     ││
│  │  - Subscribe to /gesture_command topic                  ││
│  │  - Translate commands to wheel velocities               ││
│  │  - Implement 3-second connection-loss safety STOP       ││
│  │  - Publish to /cmd_vel (motor control)                  ││
│  └─────────────────────────────────────────────────────────┘│
│              ↓ (ROS topic: /cmd_vel)                        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  TurtleBot3 Motor Drivers & Chassis                     ││
│  │  - Actuate OpenCR board based on velocity commands      ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### 🤙 Supported Gestures
| Gesture | Action | Velocity Command |
| :--- | :--- | :--- |
| **Fist** | **FORWARD** | Linear X: `0.2 m/s` |
| **Open Palm** | **STOP** | Linear X: `0.0 m/s` |
| **Thumb Left** | **TURN LEFT** | Angular Z: `0.5 rad/s` |
| **Thumb Right** | **TURN RIGHT** | Angular Z: `-0.5 rad/s` |
| **Neutral / Unsure** | **WAIT** (Filter) | Linear X: `0.0 m/s` |

---

## 🛠️ Installation & Setup

### ⚠️ Hardware & Network Prerequisites
* **Workstation:** Ubuntu 20.04 with Webcam (VM or Native).
* **Robot:** TurtleBot3 (Burger model) with Raspberry Pi 4.
* **VPN:** [Tailscale](https://tailscale.com/) must be installed on both devices to bypass AP isolation. Note the static `100.x.x.x` IPs.

### Workstation Setup
Clone this specific branch into your VM's environment:

```bash
cd ~/Desktop
git clone -b v2-cnn-pytorch [https://github.com/TFelbor/turtlebot3-gesture-detection.git](https://github.com/TFelbor/turtlebot3-gesture-detection.git)
cd turtlebot3-gesture-detection
```

Ensure your Python dependencies are met:
```bash
pip3 install torch torchvision mediapipe opencv-python scikit-learn seaborn
```

Configure your Tailscale IPs in the launch scripts:
```bash
nano scripts/robot_start.sh
# Update PI_IP and VM_IP with your specific Tailscale addresses
```

---

## 🚀 How to Run

To run the system, you will need **three terminal windows** open on your Ubuntu VM.

### Terminal 1: Start the ROS Master
This terminal coordinates all messages. Leave it running.
```bash
roscore
```

### Terminal 2: Boot the Robot Hardware
Use the provided bash script to securely SSH into the Pi over Tailscale. This automatically exports the correct ROS environment variables and launches the motor drivers and motion logic in the background.
```bash
cd ~/Desktop/turtlebot3-gesture-detection
./scripts/robot_start.sh
```

### Terminal 3: Launch the Vision System
You can choose which inference brain to use by running the corresponding launch file:

**Option A: Run the PyTorch MobileNetV2 Model**
```bash
roslaunch turtlebot3_gesture gesture_control_cnn.launch
```
**Option B: Run the MediaPipe Model**
```bash
roslaunch turtlebot3_gesture gesture_control_mediapipe.launch
```

---

## 📊 Performance Notes: CNN vs. MediaPipe
While the implementation of a custom MobileNetV2 CNN was an excellent exercise in deep learning and transfer learning, empirical testing showed that the **MediaPipe implementation remained more robust in varied environments.** Because the custom CNN dataset (1,700+ images) was captured in a specific, well-lit room, the model exhibited environmental bias (overfitting to lighting/contrast). MediaPipe, which relies on contrast-agnostic skeletal landmark detection rather than pixel-texture analysis, proved significantly more stable against dynamic backgrounds and shadows. A **Majority Vote Smoothing Filter** was implemented on the CNN node to mitigate prediction jitter, but MediaPipe remains the recommended architecture for immediate deployment. 
