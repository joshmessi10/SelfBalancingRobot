# 🤖 Self-Balancing Robot with Arduino UNO

A fully autonomous **self-balancing robot** designed using an **MPU6050 IMU**, **PID control**, and **DC motors** driven by an H-Bridge. This project demonstrates embedded real-time control, sensor fusion, and actuator feedback, showcasing a classic implementation of a **2-wheeled inverted pendulum** using affordable components and an Arduino.

---

## 🚀 Project Overview

This robot maintains its vertical balance by continuously reading its tilt angle via the **MPU6050's DMP (Digital Motion Processor)** and adjusting motor speeds with a **PID controller**. It also includes **Bluetooth-based commands** that allow the robot to move forward or backward while staying balanced.

---

## 🔧 Features

- 🎯 Real-time PID control (10ms loop)
- 🧭 Tilt estimation using MPU6050 DMP
- ⚙️ Dual DC motor control with L298N H-Bridge
- 🔧 Tunable PID constants
- ⚖️ Dynamic setpoint adjustment for movement

---

## 🧰 Hardware Components

| Component                  | Quantity | Notes                                      |
|---------------------------|----------|--------------------------------------------|
| Arduino UNO               | 1        | Main microcontroller board                 |
| MPU6050 IMU               | 1        | 6-axis accelerometer + gyroscope           |
| L298N Motor Driver        | 1        | Dual H-Bridge for controlling 2 DC motors  |
| DC Gear Motors (12V)      | 2        | Connected to wheels                        |
| Battery Pack (7.4–12V)    | 1        | Power source                               |
| Misc. parts               | -        | Wires, resistors, chassis, etc.            |

---

## 📐 System Architecture

```
[MPU6050] --> [DMP + YPR Angle] --> [PID Controller] --> [Motor Output]
```

---

## 🧠 Control Logic

### 1. Initialization
- MPU6050 DMP is initialized with calibration offsets.
- Interrupts are set for new sensor data.
- PID controller is initialized with `Kp`, `Ki`, `Kd`.

### 2. Main Loop
- When new data is available, pitch is extracted.
- PID computes motor correction.
- Motors are driven with PWM.

---

## ⚙️ Default PID Constants

```
double Kp = 70;
double Ki = 250;
double Kd = 3;
```

Tune according to your hardware configuration using trial-and-error or Ziegler–Nichols method.

## 📷 Media


https://github.com/user-attachments/assets/40e2413d-2d5f-4079-9ee2-3bc2e5ffdf85


## 👥 Credits

Josh Sebastián López Murcia  
