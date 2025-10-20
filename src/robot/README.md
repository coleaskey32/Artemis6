# Mobile Manipulator Robot - Webots Simulation with ROS2

A ROS2-enabled mobile manipulator: 4-wheeled base + 6-axis robotic arm in Webots simulation.

## 📋 Prerequisites

- Webots R2023b or later (installed via Homebrew)
- ROS2 Humble (installed via conda)
- Python 3.10+ (via conda for ROS2)

## 🏗️ Build

```bash
cd /Users/coleaskey/ros2_ws
colcon build --packages-select robot
source install/setup.zsh
```

## 🚀 Running the Simulation

### Step 1: Start Webots

1. Open Webots application
2. File → Open World
3. Navigate to: `/Users/coleaskey/ros2_ws/src/robot/worlds/robot.wbt`
4. Press **▶️ Play**

You should see:
```
🤖 Mobile Manipulator Controller Started!
   ├─ 🚗 4-Wheel Mobile Base: Ready
   └─ 🦾 6-Axis Robotic Arm: Ready

📡 Listening for ROS2 commands on port 9999
   ├─ Base control: /cmd_vel
   └─ Arm control: /arm_commands
```

### Step 2: Start Base Controller (in a new terminal)

```bash
conda activate ros_env
cd /Users/coleaskey/ros2_ws
source install/setup.zsh
ros2 run robot base_controller
```

You should see:
```
🚗 Base Controller Started!
📡 Listening to /cmd_vel topic...
✅ Connected to Webots controller
```

### Step 3: Control the Robot (in another terminal)

```bash
conda activate ros_env
cd /Users/coleaskey/ros2_ws
source install/setup.zsh
```

**Move forward:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

**Rotate in place:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

**Stop:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Drive in a circle:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

### Optional: Control the Robotic Arm

**Start the arm controller (in a new terminal):**
```bash
conda activate ros_env
cd /Users/coleaskey/ros2_ws
source install/setup.zsh
ros2 run robot arm_controller
```

**Move arm to a pose:**
```bash
ros2 topic pub /arm_joint_commands sensor_msgs/msg/JointState "{position: [0.5, 0.3, -0.5, 0.0, 0.5, 0.0]}"
```

**Return to home position:**
```bash
ros2 topic pub /arm_joint_commands sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Reach forward:**
```bash
ros2 topic pub /arm_joint_commands sensor_msgs/msg/JointState "{position: [0.0, 0.8, -1.0, 0.0, 0.5, 0.0]}"
```

**Joint Limits (radians):**
- Joint 1 (Base rotation):  -π to +π
- Joint 2 (Shoulder):       -π/2 to +π/2
- Joint 3 (Elbow):          -2.62 to +2.62
- Joint 4 (Wrist roll):     -π to +π
- Joint 5 (Wrist pitch):    -π/2 to +π/2
- Joint 6 (Wrist yaw):      -π to +π

## 🎮 Available Commands

### List Topics
```bash
ros2 topic list
```

### Monitor Commands
```bash
ros2 topic echo /cmd_vel
```

### Check Topic Info
```bash
ros2 topic info /cmd_vel
```

## 🚀 Quick Start with Launch File

Start everything at once:
```bash
# Start base control only
ros2 launch robot robot_bringup.launch.py

# Start base + arm control
ros2 launch robot robot_bringup.launch.py start_arm:=true

# Start base + square drive demo
ros2 launch robot robot_bringup.launch.py start_square:=true
```

## 🏗️ Architecture

```
┌──────────────────┐         ┌──────────────────────────┐
│ Base Controller  │ Socket  │ Webots Controller        │
│ (Conda Python)   │◄───────►│   (Homebrew Python)      │
│                  │  :9999  │                          │
│ Subscribes to    │         │ Controls:                │
│  • /cmd_vel      │         │  • 4 wheel motors        │
│                  │         │  • 6 arm joint motors    │
└──────────────────┘         └──────────────────────────┘
        ▲
        │ Socket
        │ :9999
        │
┌───────┴────────────┐
│  Arm Controller    │
│ (Conda Python)     │
│                    │
│ Subscribes to      │
│  • /arm_joint_     │
│    commands        │
└────────────────────┘
```

**Why this architecture?**
- **Webots controller** (robot_controller.py): Manages ALL hardware (4 wheels + 6 arm joints)
- **Base controller** (base_controller.py): ROS2 node for mobile base commands
- **Arm controller** (arm_controller.py): ROS2 node for arm commands
- Socket communication (port 9999) keeps Python environments separated
- Modular design: Run base-only, arm-only, or both together

## 🔧 Troubleshooting

### "Failed to connect to Webots controller"
- Make sure Webots is running and the world is loaded
- Make sure you pressed Play (▶️) in Webots
- The Webots controller must be running before starting ROS2 nodes

### "No module named 'rclpy'"
- Make sure you activated the conda environment: `conda activate ros_env`
- Make sure you sourced ROS2: `source install/setup.bash`

### Robot not moving
- Check that base_controller is connected (should see "✅ Connected to Webots controller")
- Make sure the Twist message values are non-zero
- Try stopping and restarting the controller

### Port already in use
- Kill the old Webots process: `pkill -9 webots`
- Or change the port in `robot_controller.py`, `base_controller.py`, and `arm_controller.py`

## 📁 Project Structure

```
robot/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── worlds/
│   └── robot.wbt               # Webots world: mobile manipulator
├── controllers/
│   └── robot_controller/
│       ├── robot_controller    # Bash launcher (sets WEBOTS_HOME)
│       └── robot_controller.py # Webots controller (base + arm)
├── scripts/
│   ├── base_controller.py      # ROS2 base control node
│   ├── arm_controller.py       # ROS2 arm control node
│   └── square_drive.py         # Demo: drive in a square
└── launch/
    └── robot_bringup.launch.py # Launch all nodes
```

## 🎯 Robot Parameters

### Mobile Base
- **Wheel radius:** 0.08 m
- **Wheel separation:** 0.4 m (left-right distance)
- **Base dimensions:** 0.4 m × 0.3 m × 0.1 m
- **Base mass:** 5 kg

### Robotic Arm (6-DOF)
- **Total reach:** ~0.55 m (when fully extended)
- **Upper arm length:** 0.25 m
- **Forearm length:** 0.2 m
- **Total arm mass:** ~1.5 kg
- **Gripper width:** 0.04 m (simple parallel gripper)

### Arm Specifications
| Joint | Type | Axis | Range (rad) | Range (deg) | Max Velocity |
|-------|------|------|-------------|-------------|--------------|
| 1 | Base Rotation | Z | ±π | ±180° | 2.0 rad/s |
| 2 | Shoulder | Y | ±π/2 | ±90° | 2.0 rad/s |
| 3 | Elbow | Y | ±2.62 | ±150° | 2.0 rad/s |
| 4 | Wrist Roll | Z | ±π | ±180° | 3.0 rad/s |
| 5 | Wrist Pitch | Y | ±π/2 | ±90° | 3.0 rad/s |
| 6 | Wrist Yaw | Z | ±π | ±180° | 3.0 rad/s |

## 📚 Next Steps

### Sensors & Perception
- [ ] Add Lidar sensor
- [ ] Add camera sensor
- [ ] Add odometry publishing
- [ ] Add joint state publishing for arm

### Control & Planning
- [ ] Implement inverse kinematics for arm
- [ ] Add MoveIt configuration for motion planning
- [ ] Create proper URDF/XACRO description
- [ ] Add collision detection

### Navigation
- [ ] Integrate with Nav2 stack
- [ ] Add SLAM capabilities
- [ ] Add autonomous navigation

### Real Hardware Integration
- [ ] Design and 3D print custom chassis
- [ ] Design and build 6-axis arm hardware
- [ ] Import CAD models into Webots
- [ ] Test and validate before physical build

## 🎓 What You're Learning

This project demonstrates professional robotics development workflow:

1. **Simulate First:** Complete functional robot in simulation
2. **Iterate Fast:** Test control algorithms without hardware risk
3. **Validate Design:** Ensure workspace, stability, and performance
4. **Build Once:** Order correct parts after validation
5. **Deploy Confidently:** Transfer proven software to hardware

This saves **time, money, and frustration**! 🚀

## 📄 License

MIT
