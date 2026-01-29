# random_position.py
import numpy as np

L0 = 0.0175
L1 = 0.088276
L2 = 0.12145
L3 = 0.182175

BASE_MIN = -np.pi
BASE_MAX = np.pi
SHOULDER_MIN = -np.pi/2
SHOULDER_MAX = np.pi/2
ELBOW_MIN = -np.pi/2
ELBOW_MAX = np.pi/2

def forward_kinematics(theta0, theta1, theta2):
    r = L2 * np.cos(theta1) + L3 * np.cos(theta1 + theta2)
    r_total = r + L0
    z_local = L2 * np.sin(theta1) + L3 * np.sin(theta1 + theta2)
    
    x = r_total * np.cos(theta0)
    y = r_total * np.sin(theta0)
    z = z_local + L1
    
    return x, y, z

def generate_random_position():
    theta0 = np.random.uniform(BASE_MIN, BASE_MAX)
    theta1 = np.random.uniform(SHOULDER_MIN, SHOULDER_MAX)
    theta2 = np.random.uniform(ELBOW_MIN, ELBOW_MAX)
    
    x, y, z = forward_kinematics(theta0, theta1, theta2)
    
    roll = np.random.uniform(-np.pi, np.pi)
    pitch = np.random.uniform(-np.pi, np.pi)
    yaw = np.random.uniform(-np.pi, np.pi)
    
    return x, y, z, roll, pitch, yaw

if __name__ == "__main__":
    x, y, z, roll, pitch, yaw = generate_random_position()
    
    print(f"Random reachable position:")
    print(f"{x:.6f},{y:.6f},{z:.6f},{roll:.6f},{pitch:.6f},{yaw:.6f}")
    
    print(f"\nFormatted for Arduino:")
    print(f"{x:.6f},{y:.6f},{z:.6f},{roll:.6f},{pitch:.6f},{yaw:.6f}")