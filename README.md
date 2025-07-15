# 🛸 Gaze-Assisted Drone Navigation in Webots

This repository contains a Webots simulation for comparing **manual drone control** with **gaze-assisted navigation** using a Crazyflie quadrotor. The core objective is to evaluate how human gaze input can enhance precision and efficiency in wall-following and waypoint navigation tasks.

---

## 🚀 Project Overview

- **Simulation Environment:** [Webots](https://cyberbotics.com/)
- **Robot Platform:** Bitcraze Crazyflie drone
- **Control Modes:**
  - Manual control via keyboard
  - Gaze-assisted autonomous navigation
- **Sensors Used:**
  - IMU, GPS, Camera, Gyroscope
  - Distance sensors (range_front, range_left, etc.)
- **Assistance Type:** Gaze-based yaw and pitch biasing with tunable strength

---
[▶️ Watch Demo Video on Google Drive](https://drive.google.com/file/d/1-pbvsPLCOfkpKlMhL0-xzVmgoVVs6b36/view?usp=sharing)


https://github.com/user-attachments/assets/659dc26b-82ff-4940-96dc-520b75a36161


## 📁 Folder Structure

```
webots-gaze-drone/
├── controllers/
│   ├── crazyflie_py_wallfollowing.py   # Main controller with gaze-assistance logic
│   ├── pid_controller.py               # PID velocity + altitude controller
│   └── wall_following.py               # Wall-following FSM
├── protos/
│   └── Crazyflie.proto                 # Custom Crazyflie definition
├── world/
│   └── wall_following.wbt              # Webots world file
├── data/
│   └── study-eye-tracking_...csv       # Gaze dataset for simulation
└── README.md                           # Project documentation
```

---

## 🧠 Gaze-Assisted Control Logic

- **Input:** CSV-based raw gaze data from an eye-tracking study
- **Activation:** Controlled via the `GAZE_ASSISTANCE_ENABLED` flag
- **Control Augmentation:**
  - Adjusts `yaw` based on horizontal gaze (left/right)
  - Adjusts `forward` pitch based on vertical gaze (up/down)
  - Bias strength is modulated via `GAZE_ASSISTANCE_STRENGTH`

```python
if GAZE_ASSISTANCE_ENABLED and gaze_x is not None:
    yaw_desired += GAZE_ASSISTANCE_STRENGTH * (gaze_x - 50) / 50
    forward_desired += GAZE_ASSISTANCE_STRENGTH * (50 - gaze_y) / 50
```

---

## 🎮 Keyboard Controls

| Key | Function |
|-----|----------|
| ↑ / ↓ / ← / → | Move in X-Y plane |
| Q / E         | Rotate (Yaw)      |
| W / S         | Ascend / Descend  |
| A             | Enable Autonomous Mode |
| D             | Disable Autonomous Mode |

---

## 🧪 Experimental Design

This simulation enables comparative analysis of:

- Human-in-the-loop control vs. gaze-augmented control
- Impact of real gaze data on drone path fidelity
- Precision in wall-following tasks using state machines

---

## 📊 Output

- **Live Feedback:** Position, yaw, sensor readings in real-time
- **Debug Messages:** Current control mode, gaze-based adjustments
- **Waypoint Reached Beep:** Audio cue when a target is reached

---

## 📜 License

This project is released under the [MIT License](https://opensource.org/licenses/MIT).  
Original drone control code adapted from Bitcraze examples.
