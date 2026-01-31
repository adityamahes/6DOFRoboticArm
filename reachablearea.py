import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

def monte_carlo_workspace(num_samples=50000):
    points = []
    
    for _ in range(num_samples):
        theta0 = np.random.uniform(BASE_MIN, BASE_MAX)
        theta1 = np.random.uniform(SHOULDER_MIN, SHOULDER_MAX)
        theta2 = np.random.uniform(ELBOW_MIN, ELBOW_MAX)
        
        x, y, z = forward_kinematics(theta0, theta1, theta2)
        points.append([x, y, z])
    
    return np.array(points)

if __name__ == "__main__":
    print("Generating workspace with Monte Carlo sampling...")
    print("This may take a moment...")
    
    points = monte_carlo_workspace(num_samples=50000)
    
    print(f"Generated {len(points)} points")
    print(f"X range: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m")
    print(f"Y range: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m")
    print(f"Z range: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m")
    
    fig = plt.figure(figsize=(15, 5))
    
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], 
                cmap='viridis', s=0.1, alpha=0.5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Workspace')
    ax1.set_box_aspect([1,1,1])
    
    ax2 = fig.add_subplot(132)
    ax2.scatter(points[:, 0], points[:, 1], c=points[:, 2], 
                cmap='viridis', s=0.1, alpha=0.5)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY plane)')
    ax2.set_aspect('equal')
    ax2.grid(True)
    
    ax3 = fig.add_subplot(133)
    r = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    ax3.scatter(r, points[:, 2], c=points[:, 2], 
                cmap='viridis', s=0.1, alpha=0.5)
    ax3.set_xlabel('R (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (RZ plane)')
    ax3.set_aspect('equal')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('workspace.png', dpi=300, bbox_inches='tight')
    print("\nWorkspace visualization saved as 'workspace.png'")
    plt.show()