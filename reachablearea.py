#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

class WorkspaceAnalyzer:
    def __init__(self):
        self.BASE_H = 0.052
        self.SH_OFFSET_R = 0.0175
        self.SH_H = 0.036276
        self.L1 = 0.12145
        self.L2 = 0.182175
        
        self.LIMITS = [
            (-np.pi, np.pi),   # Base
            (0.0, np.pi/2),    # Shoulder
            (0.0, np.pi/2)     # Elbow
        ]

    def forward_kinematics(self, t1, t2, t3):
        x1 = self.SH_OFFSET_R * np.cos(t1)
        y1 = self.SH_OFFSET_R * np.sin(t1)
        z1 = self.BASE_H + self.SH_H
        
        r1 = self.L1 * np.sin(t2)
        x2 = x1 + r1 * np.cos(t1)
        y2 = y1 + r1 * np.sin(t1)
        z2 = z1 + self.L1 * np.cos(t2)
        
        r2 = self.L2 * np.sin(t2 + t3)
        x3 = x2 + r2 * np.cos(t1)
        y3 = y2 + r2 * np.sin(t1)
        z3 = z2 + self.L2 * np.cos(t2 + t3)
        
        return x3, y3, z3

    def generate_hull(self, samples=5000):
        print(f"Sampling {samples} points to generate surface...")
        t1 = np.random.uniform(self.LIMITS[0][0], self.LIMITS[0][1], samples)
        t2 = np.random.uniform(self.LIMITS[1][0], self.LIMITS[1][1], samples)
        t3 = np.random.uniform(self.LIMITS[2][0], self.LIMITS[2][1], samples)
        
        x, y, z = self.forward_kinematics(t1, t2, t3)
        points = np.vstack((x, y, z)).T
        
        # ConvexHull generates the surface mesh
        hull = ConvexHull(points)
        return points, hull

def plot_shaded_workspace(points, hull):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the surface (shading)
    # Each 'simplex' is a triangle forming the outer shell
    ax.plot_trisurf(points[:,0], points[:,1], points[:,2], 
                    triangles=hull.simplices, 
                    cmap='plasma', alpha=0.3, edgecolor='none')

    # Optional: Plot a few faint points to show density
    ax.scatter(points[:,0], points[:,1], points[:,2], s=0.5, alpha=0.1, color='black')

    ax.set_title("Robot Reachable Volume (Shaded Boundary)", fontsize=14, fontweight='bold')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Normalize axes
    max_dim = np.max(points)
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(0, max_dim * 1.5)

    print(f"Volume of reach: {hull.volume:.6f} m^3")
    plt.show()

if __name__ == "__main__":
    analyzer = WorkspaceAnalyzer()
    pts, hull_data = analyzer.generate_hull(samples=10000)
    plot_shaded_workspace(pts, hull_data)