import math
import serial
import numpy as np
from ikpy.chain import Chain

# -------------------------------
# Configuration
# -------------------------------

URDF_PATH = "arm_urdf.urdf"
ACTIVE_LINKS = [False, True, True, True]

SERIAL_PORT = "COM5"      # CHANGE THIS
BAUD_RATE = 115200

POSITION_TOLERANCE = 0.01  # 1 cm

# -------------------------------
# Robot Model
# -------------------------------

arm = Chain.from_urdf_file(
    URDF_PATH,
    active_links_mask=ACTIVE_LINKS
)

# -------------------------------
# Arduino Connection
# -------------------------------

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# -------------------------------
# Inverse Kinematics
# -------------------------------

def compute_ik(target_xyz):
    """
    Computes IK for a 3D position.
    Returns joint angles (radians) or None if unreachable.
    """

    ik = arm.inverse_kinematics(
        target_position=target_xyz
    )

    # Validate with forward kinematics
    fk = arm.forward_kinematics(ik)
    reached_xyz = fk[:3, 3]

    error = np.linalg.norm(reached_xyz - target_xyz)

    if error > POSITION_TOLERANCE:
        return None

    return ik


# -------------------------------
# Arduino Command Format
# -------------------------------

def send_to_arduino(joints_rad, move_time=2.0):
    """
    Sends 3 joint angles to Arduino in degrees.
    """

    base, shoulder, elbow = [
        math.degrees(joints_rad[i]) for i in range(1, 4)
    ]

    command = (
        f"0{base:.2f} "
        f"1{shoulder:.2f} "
        f"2{elbow:.2f} "
        f"t{move_time:.2f}\n"
    )

    arduino.write(command.encode())
    print("Sent:", command.strip())


# -------------------------------
# Public API
# -------------------------------

def move_to(x, y, z, move_time=2.0):
    """
    Move end-effector to (x, y, z).
    """

    target = np.array([x, y, z])

    ik = compute_ik(target)

    if ik is None:
        print(f"UNREACHABLE TARGET: {target}")
        return False

    send_to_arduino(ik, move_time)
    return True


# -------------------------------
# Example
# -------------------------------

if __name__ == "__main__":
    move_to(0.0, 0.15, 0.25)
