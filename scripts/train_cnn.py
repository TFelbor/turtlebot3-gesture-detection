import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, models, transforms
from torch.utils.data import DataLoader
import os
import json

# 1. Setup Device - M4 VM will use CPU
device = torch.device("cpu")
print(f"Training on device: {device}")

# 2. Data Preparation
data_dir = "/home/tf/Desktop/v2/data"

# Image Augmentation: Helps the model generalize better
data_transforms = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.RandomHorizontalFlip(),
    transforms.ColorJitter(brightness=0.2, contrast=0.2), # Handles lighting changes
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# Load the dataset from your 5 folders
full_dataset = datasets.ImageFolder(data_dir, transform=data_transforms)
class_names = full_dataset.classes
print(f"Detected Classes: {class_names}")

# Split: 80% Training, 20% Validation
train_size = int(0.8 * len(full_dataset))
val_size = len(full_dataset) - train_size
train_dataset, val_dataset = torch.utils.data.random_split(full_dataset, [train_size, val_size])

# Use num_workers=0 to avoid multiprocessing errors in some VM setups
train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True, num_workers=0)
val_loader = DataLoader(val_dataset, batch_size=16, shuffle=False, num_workers=0)

# 3. Model Setup: MobileNetV2 (Lightweight for TurtleBot3)
model = models.mobilenet_v2(weights=models.MobileNet_V2_Weights.DEFAULT)

# Freeze early layers to use Transfer Learning
for param in model.parameters():
    param.requires_grad = False

# Adjust the final layer to match your 5 gestures
num_ftrs = model.classifier[1].in_features
model.classifier[1] = nn.Linear(num_ftrs, len(class_names))
model = model.to(device)

# 4. Loss and Optimization
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.classifier[1].parameters(), lr=0.001)

# 5. Training Loop
num_epochs = 10
print("Starting training...")

for epoch in range(num_epochs):
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0

    for inputs, labels in train_loader:
        inputs, labels = inputs.to(device), labels.to(device)
        
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
        _, predicted = outputs.max(1)
        total += labels.size(0)
        correct += predicted.eq(labels).sum().item()

    # Validation Phase
    model.eval()
    val_correct = 0
    val_total = 0
    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, predicted = outputs.max(1)
            val_total += labels.size(0)
            val_correct += predicted.eq(labels).sum().item()

    print(f"Epoch {epoch+1}/{num_epochs}: "
          f"Loss: {running_loss/len(train_loader):.3f} | "
          f"Acc: {100.*correct/total:.1f}% | "
          f"Val Acc: {100.*val_correct/val_total:.1f}%")

# 6. Save Model and Class Mapping
torch.save(model.state_dict(), "gesture_model_v2.pth")
with open("class_mapping.json", "w") as f:
    json.dump({i: name for i, name in enumerate(class_names)}, f)

print("Finished! Model and class mapping saved.")
