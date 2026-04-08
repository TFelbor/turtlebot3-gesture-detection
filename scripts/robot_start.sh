#!/bin/bash

# --- Tailscale Configuration ---
PI_USER="ubuntu"              # Change to "pi" if applicable
PI_IP="100.108.253.32"        # Your Pi's Forever IP
VM_IP="100.80.163.51"         # Your VM's Forever IP

echo "🔍 Checking Tailscale connection to TurtleBot3 at $PI_IP..."

if ! ping -c 1 -W 2 "$PI_IP" > /dev/null; then
    echo "❌ Error: Cannot reach the Pi. Is Tailscale running on both devices?"
    exit 1
fi

echo "✅ Connected! Routing ROS traffic through Tailscale..."

# SSH into the Pi and launch
ssh "$PI_USER@$PI_IP" << EOF
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash

    # Force ROS to use the Tailscale Tunnel
    export ROS_MASTER_URI=http://$VM_IP:11311
    export ROS_HOSTNAME=$PI_IP
    export ROS_IP=$PI_IP

    # Tell the driver which robot this is
    export TURTLEBOT3_MODEL=burger
    export LDS_MODEL=LDS-01

    echo "🟢 Starting robot hardware and motion control in the background..."
    nohup roslaunch motion_control robot_control.launch > ~/ros_bringup.log 2>&1 &
    
    echo "✅ Robot Hardware Started successfully!"
EOF

echo "🎉 Bringup complete!"
echo "➡️ Entering an interactive session on the Pi..."

# This new line connects a SECOND time, but in interactive "human" mode
ssh "$PI_USER@$PI_IP"
