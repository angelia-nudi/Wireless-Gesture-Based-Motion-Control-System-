import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split

# Load IMU Dataset
file_path = "sensor_raw.csv"
df = pd.read_csv(file_path)

# Extract IMU sensor features (gyroscope & accelerometer)
imu_columns = ["GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ"]
X = df[imu_columns].values

# Generate Q & R labels (Heuristic estimation based on motion intensity)
def estimate_Q_R(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z):
    motion_intensity = abs(gyro_x) + abs(gyro_y) + abs(gyro_z)
    if motion_intensity < 0.5:
        return 0.001, 0.1  # Low motion → low Q, high R
    elif motion_intensity < 2:
        return 0.01, 0.05  # Moderate motion
    else:
        return 0.1, 0.01  # High motion → high Q, low R

df["Q"], df["R"] = zip(*df.apply(lambda row: estimate_Q_R(row["GyroX"], row["GyroY"], row["GyroZ"],
                                                           row["AccX"], row["AccY"], row["AccZ"]), axis=1))

# Extract Q & R labels
y = df[["Q", "R"]].values  

# Normalize Data
scaler_X = MinMaxScaler()
scaler_y = MinMaxScaler()
X_scaled = scaler_X.fit_transform(X)
y_scaled = scaler_y.fit_transform(y)

# Split into Train & Test Sets
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_scaled, test_size=0.2, random_state=42)

# Convert Data to PyTorch Tensors
X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
y_train_tensor = torch.tensor(y_train, dtype=torch.float32)

# Define ANN Model
class KalmanANN(nn.Module):
    def __init__(self):
        super(KalmanANN, self).__init__()
        self.fc1 = nn.Linear(6, 16)
        self.fc2 = nn.Linear(16, 8)
        self.fc3 = nn.Linear(8, 2)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# Train ANN Model
model = KalmanANN()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.01)

for epoch in range(500):  
    optimizer.zero_grad()
    outputs = model(X_train_tensor)
    loss = criterion(outputs, y_train_tensor)
    loss.backward()
    optimizer.step()
    
    if epoch % 50 == 0:
        print(f"Epoch {epoch}: Loss = {loss.item()}")

# Save the Trained ANN Model
torch.save(model.state_dict(), "kalman_ann.pth")
print("\n ANN Model Saved as 'kalman_ann.pth'.")
