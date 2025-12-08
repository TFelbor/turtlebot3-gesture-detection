#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotionNode:
    def __init__(self):
        rospy.init_node('motion_control_node', anonymous=True)
        
        # 1. Subscriber: Listen to the VM's commands
        self.sub = rospy.Subscriber('/gesture_command', String, self.callback)
        
        # 2. Publisher: Talk to the TurtleBot motors
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Current command state
        self.current_gesture = "STOP"
        self.last_command_time = rospy.Time.now()
        
        rospy.loginfo("Motion Control Node Started. Ready to drive!")

    def callback(self, data):
        """Called every time the VM sends a gesture"""
        self.current_gesture = data.data
        self.last_command_time = rospy.Time.now()
        
        # Execute immediately
        self.move_robot()

    def move_robot(self):
        twist = Twist()
        
        # PRD Specifications:
        # GO = Linear 0.2 m/s [cite: 28]
        # STOP = 0.0 m/s [cite: 29]
        
        if self.current_gesture == "GO":
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            rospy.loginfo_throttle(1, "Moving FORWARD")
            
        elif self.current_gesture == "STOP" or self.current_gesture == "WAIT":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # No log needed for stop to keep terminal clean
            
        # SAFETY: Unknown command -> STOP
        else:
            twist.linear.x = 0.0
            
        self.cmd_vel_pub.publish(twist)

    def safety_loop(self):
        """Checks if we lost connection to the VM"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Safety Timeout: If no command for 3 seconds, STOP [cite: 33]
            time_since_last = rospy.Time.now() - self.last_command_time
            
            if time_since_last.to_sec() > 3.0:
                if self.current_gesture != "STOP":
                    rospy.logwarn("Connection Timeout! Auto-STOP triggered.")
                    self.current_gesture = "STOP"
                    self.move_robot()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MotionNode()
        node.safety_loop()
    except rospy.ROSInterruptException:
        pass
