# 🤖 Hand Robot ROS2 Control System

A complete ROS2 control system for a robotic hand with advanced gesture recognition, smooth transitions, and automatic timeout features.

## ✨ Features

- **5 Different Gestures**: open, close, thumbs_up, peace, stone
- **Smooth Transitions**: Natural 1-second interpolated movements between gestures
- **Auto-Timeout**: Returns to open state after 5 seconds of inactivity
- **Manual Control**: Send gestures on-demand via ROS2 topics
- **RViz Visualization**: Real-time 3D visualization with individual finger joints
- **15-Joint Control**: Full finger articulation with smooth animations

## 🎭 Available Gestures

| Gesture | Description | Command |
|---------|-------------|---------|
| **open** | Relaxed open hand | `'open'` |
| **close** | Normal closed fist | `'close'` |
| **thumbs_up** | Thumb extended, fingers closed 👍 | `'thumbs_up'` |
| **peace** | Index & middle fingers extended ✌️ | `'peace'` |
| **stone** | Fully closed tight fist ✊ | `'stone'` |

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy (or compatible version)
- Python 3.8+
- `ros2_control` packages
- `rviz2`

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/Shindepooja30/handrobot_ros2_control.git
cd handrobot_ros2_control
```

2. **Build the package**
```bash
colcon build --packages-select handrobot_ros2_control
source install/setup.bash
```

3. **Launch the system**
```bash
ros2 launch handrobot_ros2_control view_robot.launch.py
```

## 🎮 Usage

### Manual Gesture Control

Send gesture commands using ROS2 topics:

```bash
# Open hand (relaxed position)
ros2 topic pub /gesture std_msgs/msg/String "data: 'open'" -1

# Close hand (normal fist)
ros2 topic pub /gesture std_msgs/msg/String "data: 'close'" -1

# Thumbs up 👍
ros2 topic pub /gesture std_msgs/msg/String "data: 'thumbs_up'" -1

# Peace sign ✌️
ros2 topic pub /gesture std_msgs/msg/String "data: 'peace'" -1

# Stone (tight fist) ✊
ros2 topic pub /gesture std_msgs/msg/String "data: 'stone'" -1
```

### Example Session

```bash
# Terminal 1: Launch system
ros2 launch handrobot_ros2_control view_robot.launch.py

# Terminal 2: Send gestures
ros2 topic pub /gesture std_msgs/msg/String "data: 'peace'" -1
# Watch smooth transition to peace sign

ros2 topic pub /gesture std_msgs/msg/String "data: 'thumbs_up'" -1
# Watch smooth transition from peace to thumbs up

# Wait 5 seconds without sending anything
# Hand automatically returns to open state smoothly
```

## ⏰ Automatic Features

### Timeout Behavior
- **Duration**: 5 seconds of no gesture commands
- **Action**: Automatically returns to "open" state with smooth transition
- **Reset**: Timer resets with each new gesture command

### Smooth Transitions
- **Duration**: 1 second smooth interpolation between gestures
- **Rate**: 50Hz update rate for ultra-smooth movement
- **Type**: Linear interpolation between joint positions

## 🔧 System Architecture

### Topics
- `/gesture` - Send gesture commands (std_msgs/String)
- `/joint_states` - Joint state information for RViz
- `/hand_joint_position_controller/commands` - Direct joint commands (Float64MultiArray)

### Nodes
- `hand_controller_node` - Main gesture control with smooth transitions and timeout
- `robot_state_publisher` - Publishes robot description to RViz
- `joint_state_broadcaster` - Broadcasts joint states from ros2_control
- `controller_manager` - Manages ros2_control controllers

### Controllers
- `joint_state_broadcaster` - Publishes joint states
- `hand_joint_position_controller` - Forward command controller for joint positions

## 🎨 RViz Visualization

The hand robot appears in RViz with:
- **Individual finger segments** in different colors (blue, orange, green)
- **Real-time joint movement** synchronized with commands
- **Smooth animations** during gesture transitions
- **Base link** (yellow) and movable finger joints

## 🛠️ Technical Details

### Joint Configuration
- **15 total joints**: 1 fixed base joint + 14 movable finger joints
- **Joint names**: base_joint, joint1-bd, joint1-du, joint2-bd, joint2-dm, joint2-mu, joint3-bd, joint3-dm, joint3-mu, joint4-bd, joint4-dm, joint4-mu, joint5-bd, joint5-dm, joint5-mu
- **Joint types**: Continuous rotation joints with position control

### Gesture Definitions
Each gesture is defined as a 15-element array of joint positions:
- **Range**: 0.0 (extended) to 1.0 (fully bent)
- **Base joint**: Always 0.0 (fixed)
- **Finger joints**: Variable based on gesture

## 🔍 Troubleshooting

### Common Issues
1. **RViz shows yellow block**: Joint states not being published correctly
2. **No movement**: Check if controllers are loaded properly
3. **Jerky movement**: Should not happen with smooth transition system

### Debug Commands
```bash
# Check joint states
ros2 topic echo /joint_states

# List active controllers
ros2 control list_controllers

# Check gesture topic
ros2 topic echo /gesture
```

## 🎉 Success Indicators

When everything is working correctly, you should see:
- ✅ Individual colored finger segments in RViz (not a solid yellow block)
- ✅ Smooth 1-second transitions between gestures
- ✅ Automatic return to open state after 5 seconds of inactivity
- ✅ No error messages in terminal
- ✅ Real-time responsive gesture changes

## 📁 Project Structure

```
handrobot_ros2_control/
├── handrobot_ros2_control/
│   ├── hand_controller_node.py      # Main gesture controller
│   └── gesture_command_publisher.py # Optional gesture publisher
├── launch/
│   └── view_robot.launch.py         # Main launch file
├── config/
│   └── handrobot_controllers.yaml   # Controller configuration
├── urdf/
│   ├── handrobot.urdf              # Robot description
│   └── ros2_control/
│       └── handrobot.ros2_control.xacro # ROS2 control configuration
├── package.xml                      # Package metadata
├── setup.py                        # Python package setup
└── CMakeLists.txt                  # Build configuration
```


