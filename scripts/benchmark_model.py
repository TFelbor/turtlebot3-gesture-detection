#!/usr/bin/env python3
import torch
import torch.nn as nn
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, classification_report
import json
import numpy as np

print("🚀 Starting CNN Benchmarking Tool...")

# --- 1. Setup Paths & Device ---
device = torch.device("cpu")
data_dir = "/home/tf/Desktop/v2/data"
model_path = "/home/tf/Desktop/v2/gesture_model_v2.pth"
mapping_path = "/home/tf/Desktop/v2/class_mapping.json"

# --- 2. Load Class Mapping ---
with open(mapping_path, 'r') as f:
    class_mapping = {int(k): v for k, v in json.load(f).items()}
class_names = [class_mapping[i] for i in range(len(class_mapping))]
print(f"📂 Loaded Classes: {class_names}")

# --- 3. Data Preprocessing ---
# (We only need resizing and normalizing for evaluation, no random flipping here)
eval_transforms = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

print("🖼️ Loading dataset...")
dataset = datasets.ImageFolder(data_dir, transform=eval_transforms)
loader = DataLoader(dataset, batch_size=16, shuffle=False, num_workers=0)

# --- 4. Load the PyTorch Model ---
print("🧠 Building MobileNetV2 and loading weights...")
model = models.mobilenet_v2()
num_ftrs = model.classifier[1].in_features
model.classifier[1] = nn.Linear(num_ftrs, len(class_names))

model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
model.eval()

# --- 5. Run Inference ---
print("🔍 Running images through the CNN... (This may take a minute)")
y_true = []
y_pred = []

with torch.no_grad():
    for inputs, labels in loader:
        outputs = model(inputs)
        _, preds = torch.max(outputs, 1)
        
        y_true.extend(labels.numpy())
        y_pred.extend(preds.numpy())

# --- 6. Generate Classification Report (Console) ---
print("\n" + "="*50)
print("📊 CLASSIFICATION REPORT")
print("="*50)
report = classification_report(y_true, y_pred, target_names=class_names)
print(report)

# --- 7. Generate & Save Confusion Matrix Plot (For LaTeX) ---
print("🎨 Drawing Confusion Matrix...")
cm = confusion_matrix(y_true, y_pred)

plt.figure(figsize=(10, 8))
sns.set_theme(font_scale=1.2) # Make fonts readable for the report
ax = sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                 xticklabels=class_names, yticklabels=class_names, 
                 cbar_kws={'label': 'Number of Images'})

plt.xlabel('Predicted Gesture', fontsize=14, fontweight='bold')
plt.ylabel('Actual (True) Gesture', fontsize=14, fontweight='bold')
plt.title('MobileNetV2 Validation Accuracy', fontsize=16, pad=20)
plt.xticks(rotation=45)
plt.yticks(rotation=0)

# Save the plot to the same folder so you can grab it for Overleaf/LaTeX
output_file = "/home/tf/Desktop/v2/training_benchmark.png"
plt.tight_layout()
plt.savefig(output_file, dpi=300) # High resolution for PDF reports
print(f"✅ Success! Plot saved to: {output_file}")
