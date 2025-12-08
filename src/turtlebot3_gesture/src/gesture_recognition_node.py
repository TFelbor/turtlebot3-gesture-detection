#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
from std_msgs.msg import String

class GestureNode:
    def __init__(self):
        # 1. Initialize ROS Node
        rospy.init_node('gesture_recognition_node', anonymous=True)

        # 2. Create Publisher (Topic: /gesture_command)
        self.pub = rospy.Publisher('/gesture_command', String, queue_size=10)

        # 3. Setup MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # 4. Camera Setup
        self.cap = cv2.VideoCapture(0)

        rospy.loginfo("Gesture Recognition Node Started. Waiting for hands...")

    def get_gesture(self, hand_landmarks):
        """
        Classifies the hand pose into a command string.
        """
        # Landmarks reference: 
        # 0=WRIST, 4=THUMB_TIP, 8=INDEX_TIP, 12=MIDDLE_TIP, etc.
        
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        
        # Check fingers (Index, Middle, Ring, Pinky)
        # If tip is below the middle joint (PIP), it's closed.
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        
        fingers_open = []
        for i in range(4):
            if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[pips[i]].y:
                fingers_open.append(1)
            else:
                fingers_open.append(0)
        
        total_fingers = sum(fingers_open)
        
        # LOGIC TREE
        
        # 1. STOP: Open Palm (All 4 fingers open)
        if total_fingers >= 4:
            return "STOP"
            
        # 2. LEFT/RIGHT: Fist-like (fingers closed) BUT thumb is sticking out
        elif total_fingers == 0:
            # Check Thumb Orientation (X-axis)
            # Threshold: How far the thumb must be from the wrist to count
            threshold = 0.1 
            
            if thumb_tip.x < wrist.x - threshold:
                return "LEFT"  # Thumb is significantly to the left of the wrist
            elif thumb_tip.x > wrist.x + threshold:
                return "RIGHT" # Thumb is significantly to the right of the wrist
            else:
                return "GO"    # Thumb is close to wrist (Standard Fist)
                
        # 3. GO: Standard Fist (0 fingers, thumb not extended far)
        # (Covered by the 'else' above)
        
        return "WAIT"

    def start(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown() and self.cap.isOpened():
            success, img = self.cap.read()
            if not success:
                continue

            img = cv2.flip(img, 1)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)
            
            command = "WAIT"

            if results.multi_hand_landmarks:
                for hand_lms in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                    
                    # USE NEW FUNCTION HERE
                    command = self.get_gesture(hand_lms)
            
            self.pub.publish(command)
            
            cv2.putText(img, f"CMD: {command}", (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Gesture Control", img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = GestureNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
