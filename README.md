# 6-DOF Robotic Manipulator (v1)

**A 6-Degree-of-Freedom robotic arm built from scratch to test custom Inverse Kinematics (IK) algorithms and real-time motion planning.**

## Motivation

The goal was not to assemble a kit, but to engineer the entire stack—from the mechanical CAD design to the low-level motor control firmware. The primary challenge was implementing a custom **Inverse Kinematics solver** to translate 3D Cartesian coordinates  into specific joint angles  in real-time.

Future iterations will serve as a hardware testbed for **Brain-Computer Interface (BCI)** integration, translating EEG signals into physical manipulation.

## Tech Stack

### Software

* **Inverse Kinematics Solver:** Python (NumPy/SciPy) for matrix transformations.
* **Visualization:** Matplotlib for real-time 3D simulation.
* **Firmware:** C++ (Arduino) for servo control and serial communication.
* **Communication:** PySerial (UART) for PC-to-Microcontroller data stream.

### Hardware

* **Controller:** Arduino Mega.
* **Actuators:** MG996R (High Torque)
* **Chassis:** Custom designed in SolidWorks, 3D printed in PLA/PETG.
* **Power:**  6V 1.5A PSU (Crucial for preventing brownouts).

## The Math (Inverse Kinematics)

This project uses [Geometric / Analytical / Numerical] methods to solve the IK problem.

Key challenges solved:

1. **Singularity Management:** Handling edge cases where the Jacobian determinant approaches zero (e.g., "gimbal lock" or full extension).
2. **Joint Limits:** Software-clamping angles to prevent physical self-collision.
3. **Coordinate Transformation:** Geometric transformation maps the end-effector to the base by summing link vectors via trigonometry for position and accumulating joint angles for orientation.

## ⚡ Technical Challenges & Solutions

| Challenge | Solution |
| --- | --- |
| **PWM Jitter** | Servos were shaking due to signal noise. Implemented a low-pass filter in software and added capacitor smoothing on the power rail. |
| **Torque Sag** | The elbow joint struggled against gravity at full extension. Added a counterweight/spring assist (or upgraded motor) to compensate. |
| **Latency** | Serial communication lag caused "stuttering." Optimized the baud rate to 115200 and packed data into binary structs instead of parsing strings. |

## Future Roadmap

* [ ] **Gripper Upgrade:** Designing a 3-finger gripper inspired by the Tesla Optimus hand.
* [ ] **Computer Vision:** Integrating OpenCV to track objects for autonomous pick-and-place.
* [ ] **BCI Integration:** Interfacing with OpenBCI hardware to control the end-effector via SSVEP or Motor Imagery signals.

## Repository Structure

```bash
├── CAD/                 # SolidWorks/STL files for printing
├── Arduino Code/            # Arduino/C++ code for the controller
├── Kinematics/          # Python scripts for IK solver & Simulation
├── Docs/                # Denavit-Hartenberg parameter tables & schematics
└── README.md

```

## Setup & Usage

1. **Flash Firmware:** Upload the arduino code to the microcontroller.
2. **Install Dependencies:**
```bash
pip install numpy pyserial matplotlib

```


3. **Run Controller:**
```bash
python Kinematics/main_controller.py

```


---
