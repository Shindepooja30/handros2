
# 🤖 Hand Robot ROS2 Control System

A complete ROS2 control system for a robotic hand with advanced gesture recognition, smooth transitions, automatic timeout features, and real servo motor actuation using Thonny (MicroPython on Raspberry Pi Pico).

---

## ✨ Features

* **5 Different Gestures**: open, close, thumbs\_up, peace, stone
* **Smooth Transitions**: Natural 1-second interpolated movements between gestures
* **Auto-Timeout**: Returns to open state after 5 seconds of inactivity
* **Manual Control**: Send gestures on-demand via ROS2 topics
* **RViz Visualization**: Real-time 3D visualization with individual finger joints
* **15-Joint Control**: Full finger articulation with smooth animations
* **Hardware Servo Support**: Control real servo motors via Thonny + MicroPython on Raspberry Pi Pico

---

## 🎭 Available Gestures

| Gesture        | Description                        | Command       |
| -------------- | ---------------------------------- | ------------- |
| **open**       | Relaxed open hand                  | `'open'`      |
| **close**      | Normal closed fist                 | `'close'`     |
| **thumbs\_up** | Thumb extended, fingers closed 👍  | `'thumbs_up'` |
| **peace**      | Index & middle fingers extended ✌️ | `'peace'`     |
| **stone**      | Fully closed tight fist ✊          | `'stone'`     |

---

## 🚀 Quick Start

### Prerequisites

* ROS2 Jazzy (or compatible version)
* Python 3.8+
* `ros2_control` packages
* `rviz2`

### Installation

```bash
git clone https://github.com/Shindepooja30/handros2.git
cd handros2
colcon build --packages-select handrobot_ros2_control
source install/setup.bash
```

### Launch the system

```bash
ros2 launch handrobot_ros2_control view_robot.launch.py
```

---

## 🎮 Usage

### Manual Gesture Control

```bash
ros2 topic pub /gesture std_msgs/msg/String "data: 'peace'" -1
ros2 topic pub /gesture std_msgs/msg/String "data: 'thumbs_up'" -1
```

Gestures transition smoothly and return to **open** after 5 seconds of inactivity.

---

## ⏰ Automatic Features

* **Timeout**: 5 s inactivity → returns to `'open'` smoothly
* **Transition**: 1 s linear interpolation at 50 Hz
* **Safety**: All joints clamped between `0.0–1.0`
---

## 🎨 RViz Visualization

* Colored finger segments, real-time joint animation, base link reference
* Smooth transitions between gestures
---

## ⏱️ Latency & Timeout Verification

| Test       | Expected | Measured | Notes                       |
| ---------- | -------- | -------- | --------------------------- |
| Transition | 1.0 s    | 0.98 s   | 50 Hz, linear interpolation |
| Timeout    | 5.0 s    | 5.02 s   | Auto-return to `'open'`     |

---

## 🛡️ Safety & Limits

* Joint values clamped to `[0.0, 1.0]`
* Max per-step delta capped → prevents velocity spikes

---

## 🔌 Hardware Actuation (Pico + Thonny)

This project supports **real servo actuation** with Raspberry Pi Pico.

1. Flash Pico with **MicroPython**.
2. Connect servos to GPIOs + external 5 V (share GND).
3. Run servo control code in Thonny.
4. Run `pico_servo_bridge.py` on terminal:

```bash
python3 pico_servo_bridge.py --port /dev/ttyACM0 --baud 115200
```

Now ROS2 gestures drive both RViz **and** real servos.

---

## 🧪 Repro & Logs

```bash
ros2 launch handrobot_ros2_control view_robot.launch.py
for g in open peace thumbs_up close stone; do
  ros2 topic pub /gesture std_msgs/msg/String "data: '$g'" -1
  sleep 2
done
ros2 topic echo /joint_states > logs/joint_states.txt
```

---

## 📝 Reflection & Next Steps

**Challenges faced**
* Avoiding jitter at 50 Hz updates
* Designing clean gesture→joint dictionary
* Synchronizing timeout with ongoing transitions

**Future work**

* Spline easing (ease-in/out) instead of linear
* Controlling it with EMG sensors

**Known limits**

* Evaluated mainly in RViz, limited hardware testing
