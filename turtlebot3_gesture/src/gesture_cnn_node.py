#!/usr/bin/env python3
import rospy
import cv2
import torch
from torchvision import transforms, models
import torch.nn as nn
from PIL import Image
from std_msgs.msg import String
import json
import collections

class GestureCNNNode:
    def __init__(self):
        rospy.init_node('gesture_cnn_node', anonymous=True)
        self.pub = rospy.Publisher('/gesture_command', String, queue_size=10)
        
        # --- 1. Load Parameters ---
        model_path = rospy.get_param('~model_path', '/home/tf/Desktop/v2/gesture_model_v2.pth')
        mapping_path = '/home/tf/Desktop/v2/class_mapping.json' 
        
        # --- 2. Load Class Mapping ---
        try:
            with open(mapping_path, 'r') as f:
                self.class_mapping = {int(k): v for k, v in json.load(f).items()}
        except Exception as e:
            rospy.logerr(f"Failed to load class mapping: {e}")
            self.class_mapping = {0: 'GO', 1: 'STOP', 2: 'LEFT', 3: 'RIGHT', 4: 'WAIT'}

        # --- 3. Load PyTorch Model ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Building MobileNetV2 skeleton on {self.device}...")
        
        self.model = models.mobilenet_v2()
        num_ftrs = self.model.classifier[1].in_features
        self.model.classifier[1] = nn.Linear(num_ftrs, len(self.class_mapping))
        
        rospy.loginfo("Loading trained weights...")
        state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model = self.model.to(self.device)
        self.model.eval() 

        # --- 4. Standard CNN Image Preprocessing ---
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # --- 5. Smoothing Filter Setup ---
        self.history_length = 5
        self.prediction_history = collections.deque(maxlen=self.history_length)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        rospy.loginfo("Gesture CNN Node Started. Camera active.")

    def get_smoothed_prediction(self, new_prediction):
        """Applies a majority-vote smoothing filter."""
        self.prediction_history.append(new_prediction)
        
        # Count occurrences of each prediction in the sliding window
        counts = collections.Counter(self.prediction_history)
        most_common_pred, count = counts.most_common(1)[0]
        
        # Confidence Threshold: Must be >60% of the window
        if count >= (self.history_length * 0.6):
            return most_common_pred
        else:
            return "WAIT" # Uncertain, do nothing

    def start(self):
        rate = rospy.Rate(15) # 15 Hz is a good balance for CNN + VM
        while not rospy.is_shutdown() and self.cap.isOpened():
            success, img = self.cap.read()
            if not success:
                continue

            img = cv2.flip(img, 1)
            
            # Convert OpenCV (BGR) to PIL (RGB) for PyTorch
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(img_rgb)
            
            # Predict
            input_tensor = self.transform(pil_img).unsqueeze(0).to(self.device)
            with torch.no_grad():
                outputs = self.model(input_tensor)
                _, predicted_idx = torch.max(outputs, 1)
                
            raw_prediction = self.class_mapping[predicted_idx.item()]
            
            # Apply Smoothing Filter
            final_command = self.get_smoothed_prediction(raw_prediction)
            
            # Publish and Visualize
            self.pub.publish(final_command)
            cv2.putText(img, f"CMD: {final_command} (Raw: {raw_prediction})", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("CNN Gesture Control", img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = GestureCNNNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
