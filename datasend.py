import math
import serial
import numpy as np
from ikpy.chain import Chain


URDF_PATH = "arm_urdf.urdf"
ACTIVE_LINKS = [False, True, True, True]

SERIAL_PORT = "COM5"      # CHANGE THIS
BAUD_RATE = 115200

POSITION_TOLERANCE = 0.01  # 1 cm

arm = Chain.from_urdf_file(
    URDF_PATH,
    active_links_mask=ACTIVE_LINKS
)
print("URDF IS LOADED RIGHT NOW!!")

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def compute_ik(target_xyz):
    # Returns joint angles (radians) or None if unreachable

    ik = arm.inverse_kinematics(
        target_position=target_xyz
    )


    fk = arm.forward_kinematics(ik)
    reached_xyz = fk[:3, 3]

    error = np.linalg.norm(reached_xyz - target_xyz)

    if error > POSITION_TOLERANCE:
        return None

    return ik


def send_to_arduino(joints_rad, move_time=2.0):

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



def move_to(x, y, z, move_time=2.0):

    target = np.array([x, y, z])

    ik = compute_ik(target)

    if ik is None:
        print(f"UNREACHABLE TARGET: {target}")
        return False

    send_to_arduino(ik, move_time)
    return True



if __name__ == "__main__":
    move_to(0.0, 0.15, 0.25)
