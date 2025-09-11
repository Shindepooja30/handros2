Got it 👍 I’ll extend your README with the **MQTT + EMG sensor integration** part, latency/safety details, and usage examples so that it’s clean and consistent with your current style.

Here’s the updated version with the new sections added:

---

# 🤖 Hand Robot ROS2 Control System

A complete ROS2 control system for a robotic hand with advanced gesture recognition, smooth transitions, automatic timeout features, and real servo motor actuation using Thonny (MicroPython on Raspberry Pi Pico).

---

## ✨ Features

* **3 Different Gestures**: open, close, stone
* **Smooth Transitions**: Natural 1-second interpolated movements between gestures
* **Auto-Timeout**: Returns to open state after 5 seconds of inactivity
* **Manual Control**: Send gestures on-demand via ROS2 topics
* **RViz Visualization**: Real-time 3D visualization with individual finger joints
* **15-Joint Control**: Full finger articulation with smooth animations
* **EMG + MQTT Integration**: Control gestures in RViz and real servos using muscle signals via MQTT

---

## 🎭 Available Gestures

| Gesture        | Description                        | Command       |
| -------------- | ---------------------------------- | ------------- |
| **open**       | Relaxed open hand                  | `'open'`      |
| **close**      | Normal closed fist                 | `'close'`     |
| **stone**      | Fully closed tight fist ✊          | `'stone'`     |

---

## 🚀 Quick Start

### Prerequisites

* ROS2 Jazzy 
* Python 3.8+
* `ros2_control` packages
* `rviz2`
* MQTT broker (e.g., Mosquitto)
* Raspberry Pi Pico W with MicroPython

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

Whenever I will flex it will send data to rviz using mqtt communication and the hand will move as per reading.

Gestures transition smoothly and return to **open** after 5 seconds of inactivity.

---

## 🌐 EMG + MQTT Control

This project also supports **real-time EMG gesture recognition** using MQTT:

1. **EMG Data Acquisition**

   * EMG sensor connected to Raspberry Pi Pico 2W
   * Pico reads analog EMG signals and publishes via MQTT

2. **Bridge Node**

   * An MQTT-to-ROS2 bridge subscribes to EMG data
   * Translates signals → gestures → publishes to `/gesture` topic

3. **Gesture Mapping**

   * **Single flex** → `'close'`
   * **Relax (no flex)** → `'open'`
   * **Double flex** → `'stone'`

```bash
# Run the bridge node
 python3 mqtt_ros2_bridge.py
```

Now, flexing your hand controls both **RViz** visualization.

---

## ⏱️ Latency & Input Handling

* **Latency chain**: `EMG → MQTT → ROS2 bridge → gesture → motion`

  * Avg latency measured: \~120 ms
* **Invalid/noisy input**

  * Bridge filters out noise & unrecognized signals
  * Defaults to `'open'` when uncertain
* **Safety considerations**

  * All joint values clamped to `[0.0, 1.0]`
  * Invalid MQTT data ignored
  * Servo velocity capped to prevent hardware damage

---

## 🖥️ Example Input/Output

### Input (from EMG via MQTT)
Published EMG:65535
Published EMG:654
Published EMG:56578
Published EMG:65535
Published EMG:65535
Published EMG:34356
Published EMG:45446
Published EMG:655
Published EMG:543

### Bridge Output (ROS2 topic `/gesture`)
Mqtt - emg :65535(hand opens)
mqtt - emg :54322 or less than 65535(hand closes)
Mqtt - emg :344 or less (stone)`

### Result

* Hand in RViz transitions smoothly to **close** gesture and more.

---
## 📝 Reflection & Next Steps

**Challenges faced**

* Avoiding jitter at 50 Hz updates
* Designing clean gesture→joint dictionary
* Synchronizing timeout with ongoing transitions
* Mapping noisy EMG input to discrete gestures

**Future work**

* Controlling with more advanced EMG classifiers
* Multi-user calibration for robustness

**Known limits**

* Evaluated mainly in RViz, limited hardware testing
* Simple threshold-based EMG mapping, not ML-based yet

---
