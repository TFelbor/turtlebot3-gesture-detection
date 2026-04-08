# 🐢 TurtleBot3 Gesture Control (ROS Noetic)

This repository contains a distributed ROS Noetic pipeline that allows a user to control a TurtleBot3 Burger using real-time hand gestures. 

As of the **Fall Semester Update (`v2-cnn-pytorch` branch)**, this project features two selectable vision architectures:
1. **MediaPipe Version:** A heuristic, logic-based approach using Google's MediaPipe hand landmark detection.
2. **PyTorch CNN Version:** A custom, data-driven approach using Transfer Learning on a MobileNetV2 Convolutional Neural Network.

## 🏗️ System Architecture
The system is divided into three primary components communicating over a secure VPN tunnel:
* **The Brain (Remote VM):** An Ubuntu virtual machine that processes webcam frames, runs the heavy deep learning/computer vision models, and publishes `std_msgs/String` commands.
* **The Spine (Tailscale VPN):** A peer-to-peer overlay network that bypasses local Wi-Fi router AP isolation, allowing bidirectional ROS communication.
* **The Body (Raspberry Pi & OpenCR):** The TurtleBot3 hardware that subscribes to the gesture commands and translates them into `/cmd_vel` motor velocities.

---

## 🛠️ Prerequisites & Dependencies
* **OS:** Ubuntu 20.04 (VM) and Ubuntu 20.04 Server (Raspberry Pi)
* **Middleware:** ROS Noetic
* **Python Packages (VM):** `torch`, `torchvision`, `mediapipe`, `opencv-python`, `scikit-learn`, `seaborn`

---

## 🌐 Networking Setup (Tailscale)
A major component of this project is the Tailscale VPN integration, which guarantees connection stability regardless of dynamic local IPs or university/hotspot network restrictions.

1. Install [Tailscale](https://tailscale.com/) on both your Ubuntu VM and the Raspberry Pi.
2. Authenticate both devices to the same Tailscale account.
3. Note the static `100.x.x.x` IP addresses assigned to both devices.
4. Update the bash scripts (`robot_start.sh` and `robot_connect.sh`) on your VM with these IPs:
   ```bash
   PI_IP="100.x.x.x"  # Your Pi's Tailscale IP
   VM_IP="100.y.y.y"  # Your VM's Tailscale IP
   ```

---

## 🚀 Installation
Clone this specific branch into your VM's catkin workspace:

```bash
cd ~/catkin_ws/src
git clone -b v2-cnn-pytorch https://github.com/TFelbor/turtlebot3-gesture-detection.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
*(Note: The custom training dataset is omitted from this repository due to size constraints, but the trained PyTorch weights (`gesture_model_v2.pth`) are included).*

---

## 🎮 Running the Robot

To run the system, you will need **three terminal windows** open on your VM.

### Terminal 1: Start the ROS Master
```bash
roscore
```

### Terminal 2: Boot the Robot Hardware
Use the provided bash script to securely SSH into the Pi over Tailscale and launch the motor drivers and motion logic in the background.
```bash
cd ~/Desktop/v2
./robot_start.sh
```

### Terminal 3: Launch the Vision System
You can choose which brain to use by running the corresponding launch file:

**Option A: Run the PyTorch CNN Model**
```bash
roslaunch turtlebot3_gesture gesture_control_cnn.launch
```
**Option B: Run the MediaPipe Model**
```bash
roslaunch turtlebot3_gesture gesture_control_mediapipe.launch
```

---

## 🤙 Gesture Command Dictionary

| Gesture | Action | Velocity Command |
| :--- | :--- | :--- |
| **Fist** | **GO** (Forward) | Linear X: `0.2 m/s` |
| **Open Palm** | **STOP** | Linear X: `0.0 m/s` |
| **Thumb Left** | **LEFT** (Rotate) | Angular Z: `0.5 rad/s` |
| **Thumb Right** | **RIGHT** (Rotate) | Angular Z: `-0.5 rad/s` |
| **Neutral / Unsure** | **WAIT** (Filter) | Linear X: `0.0 m/s` |

*(Note: The `motion_control_node` includes a built-in safety timeout. If the robot loses connection to the VM or receives no commands for 3 seconds, it will automatically STOP).*

---

## 📊 Performance Notes: CNN vs. MediaPipe
While the implementation of a custom MobileNetV2 CNN was an excellent exercise in deep learning, empirical testing showed that the **MediaPipe implementation remained more robust in varied environments.** Because the custom CNN dataset was captured in a specific, well-lit room, the model exhibited environmental bias (overfitting to lighting/contrast). MediaPipe, which relies on contrast-agnostic skeletal landmark detection rather than pixel-texture analysis, proved significantly more stable against dynamic backgrounds and shadows. A **Majority Vote Smoothing Filter** was implemented on the CNN node to mitigate prediction jitter, but MediaPipe remains the recommended architecture for immediate deployment. Future CNN improvements require aggressive dataset diversification and image augmentation.
