import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from scipy.optimize import minimize
import serial
import time

DATA_PORT = 'COM8' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(DATA_PORT, BAUD_RATE, timeout=0.05)
    print(f"Connected {DATA_PORT}")
except Exception as e:
    ser = None
    print(f"FAILED {DATA_PORT}: {e}")

class RobotArm:
    def __init__(self):
        self.BASE_H = 0.052
        self.SH_OFFSET_R = 0.0175
        self.SH_H = 0.036276
        self.L1 = 0.12145
        self.L2 = 0.182175
        # Limits: Base (-180 to 180), Shoulder (0 to 90), Elbow (0 to 90)
        self.LIMITS = [(-np.pi, np.pi), (0.0, np.pi/2), (0.0, np.pi/2)]

    def forward_kinematics(self, angles):
        t1, t2, t3 = angles

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

    
        return np.array([x3, y3, z3])

    def get_link_positions(self, angles):
        t1, t2, t3 = angles
        p0 = np.array([0, 0, 0])
        p1 = np.array([0, 0, self.BASE_H])
        x1 = self.SH_OFFSET_R * np.cos(t1); y1 = self.SH_OFFSET_R * np.sin(t1); z1 = self.BASE_H + self.SH_H
        p2 = np.array([x1, y1, z1])
        r1 = self.L1 * np.sin(t2); x2 = x1 + r1 * np.cos(t1); y2 = y1 + r1 * np.sin(t1); z2 = z1 + self.L1 * np.cos(t2)
        p3 = np.array([x2, y2, z2])
        r2 = self.L2 * np.sin(t2 + t3); x3 = x2 + r2 * np.cos(t1); y3 = y2 + r2 * np.sin(t1); z3 = z2 + self.L2 * np.cos(t2 + t3)
        p4 = np.array([x3, y3, z3])
        return np.array([p0, p1, p2, p3, p4])

    def inverse_kinematics(self, target):
        def cost(angles): return np.linalg.norm(self.forward_kinematics(angles) - target)
        res = minimize(cost, [0, 0, 0], method='SLSQP', bounds=self.LIMITS, tol=1e-6)
        return res.x, np.linalg.norm(self.forward_kinematics(res.x) - target)

        
        # # USING NORMAL GEOMETRY:
        # x, y, z = target

        # t1 = np.arctan2(y, x)

        # r = np.sqrt(x**2 + y**2) - self.SH_OFFSET_R
        # z_p = z - (self.BASE_H + self.SH_H)

        # #elbow
        # L1, L2 = self.L1, self.L2
        # D = (r**2 + z_p**2 - L1**2 - L2**2) / (2 * L1 * L2)

        # if np.abs(D) > 1.0:
        #     return np.array([0.0, 0.0, 0.0]), np.inf  # unreachable

        # t3 = np.arccos(D)  # elbow-down solution

        # #shoulder
        # phi = np.arctan2(r, z_p)
        # psi = np.arctan2(L2 * np.sin(t3), L1 + L2 * np.cos(t3))
        # t2 = phi - psi
        # angles = np.array([t1, t2, t3])

        # #joint limits check
        # for i, (low, high) in enumerate(self.LIMITS):
        #     if not (low <= angles[i] <= high):
        #         return angles, np.inf

        # err = np.linalg.norm(self.forward_kinematics(angles) - target)
        # return angles, err

        

class Interactive3DRobot:
    def __init__(self, robot):
        self.robot = robot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.25)
        
        self.sl_x = Slider(plt.axes([0.2, 0.15, 0.6, 0.03]), 'X', -0.4, 0.4, valinit=0.1)
        self.sl_y = Slider(plt.axes([0.2, 0.10, 0.6, 0.03]), 'Y', -0.4, 0.4, valinit=0.1)
        self.sl_z = Slider(plt.axes([0.2, 0.05, 0.6, 0.03]), 'Z', 0.0, 0.5, valinit=0.2)
        
        for s in [self.sl_x, self.sl_y, self.sl_z]: s.on_changed(self.update)
        self.update(None)

    def update(self, val):
        target = np.array([self.sl_x.val, self.sl_y.val, self.sl_z.val])
        angles, error = self.robot.inverse_kinematics(target)
        
        status_ok = error < 0.02
        deg_base = int(np.degrees(angles[0]) + 90)
        deg_sh   = int(np.degrees(angles[1]) + 90)
        deg_el   = int(np.degrees(angles[2]) + 90)
        
        if status_ok and ser:
            # Format: "B90,S90,E90\n"
            msg = f"B{deg_base},S{deg_sh},E{deg_el}\n"
            ser.write(msg.encode())
            print(f"SENT: {msg.strip()}")

        pts = self.robot.get_link_positions(angles)
        self.ax.clear()
        self.ax.set_xlim([-0.4, 0.4]); self.ax.set_ylim([-0.4, 0.4]); self.ax.set_zlim([0, 0.5])
        self.ax.set_title(f"IK Error: {error:.4f}m | {'OK' if status_ok else 'OUT OF REACH'}")
        self.ax.plot3D(pts[:,0], pts[:,1], pts[:,2], 'o-', linewidth=6, color='#2c3e50')
        self.ax.scatter(target[0], target[1], target[2], color='red', s=100)
        self.fig.canvas.draw_idle()

if __name__ == "__main__":
    bot = RobotArm()
    ui = Interactive3DRobot(bot)
    plt.show()