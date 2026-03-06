# PTZ Camera Control System - Dual Controller Setup

## Overview

The PTZ system now supports two control modes that can be dynamically switched:

### 🎮 **Available Controllers**
1. **Position Controller**: Controls absolute joint positions
2. **Velocity Controller**: Controls joint velocities

### 🔄 **Automatic Switch System**
- **Velocity Mode**: Automatically activated when receiving commands on `/cmd/velocity`
- **Position Mode**: Automatically activated when receiving commands via action server

## 🚀 System Launch

### 1. Start rover simulation:
```bash
ros2 launch rover_gazebo rover_garden.launch.py
```

### 2. Spawn PTZ camera:
```bash
ros2 launch ptz_gz_sim spawn_ptz_in_rover_world.launch.py ptz_x:=3.0 ptz_y:=1.0 ptz_z:=2.0
```

## 📡 Control Interfaces

### **Velocity Control** (Continuous control)
```bash
# Via topic (automatically activates velocity controller)
ros2 topic pub /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.5, tilt: -0.2, zoom: 0.0}"

# Stop movement
ros2 topic pub /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.0, tilt: 0.0, zoom: 0.0}"

# DIRECT velocity controller control
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.3, -0.2]"
```

### **Position Control** (Target movement)
```bash
# Via action server (automatically activates position controller)
ros2 action send_goal /move_ptz/position_abs ptz_action_server_msgs/action/PtzMove "{ptz: {pan: 1.0, tilt: -0.5, zoom: 1.0}}"

# DIRECT position controller control
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, -0.5]"
```

### **State Monitoring**
```bash
# PTZ state (includes active mode)
ros2 topic echo /ptz_state

# Joint states
ros2 topic echo /joint_states

# List active controllers
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## 🧪 Automated Testing

### Complete Test Script:
```bash
ros2 run ptz_gz_sim test_ptz_controllers.py
```

### Individual Tests (from Python shell):
```python
# In test_ptz_controllers.py node
tester.test_velocity()     # Test velocity via fake_axis_camera
tester.test_position()     # Test position via action server
tester.test_direct_vel()   # Test direct velocity controller
tester.test_direct_pos()   # Test direct position controller
```

## ⚙️ Manual Controller Switch (Advanced)

```bash
# Switch to velocity controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [velocity_controller], deactivate_controllers: [position_controller], strictness: 1}"

# Switch to position controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [position_controller], deactivate_controllers: [velocity_controller], strictness: 1}"
```

## 📊 System Monitoring

### Controller Status:
```bash
# List all controllers
ros2 control list_controllers

# Info on specific controller
ros2 control list_hardware_interfaces
```

### Main Topics:
- `/ptz_state` - Current PTZ state + active mode
- `/cmd/velocity` - Velocity input (auto-switch to velocity mode)
- `/move_ptz/position_abs` - Position action (auto-switch to position mode)
- `/velocity_controller/commands` - Direct velocity controller input
- `/position_controller/commands` - Direct position controller input
- `/joint_states` - Actual joint feedback

## 🔧 Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   /cmd/velocity │───▶│  fake_axis_camera │───▶│velocity_controller│
└─────────────────┘    │                  │    └─────────────────┘
                       │   (auto-switch)  │
┌─────────────────┐    │                  │    ┌─────────────────┐
│  action server  │───▶│                  │───▶│position_controller│
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────┐
                       │    PTZ       │
                       │  Simulation  │
                       └──────────────┘
```

## ✅ Implemented Features

- ✅ **Auto-switch** between controllers based on input type
- ✅ **Velocity control** continuous and responsive
- ✅ **Position control** with specific targets
- ✅ **State feedback** with active mode
- ✅ **Direct controller access** for advanced testing
- ✅ **Action server** for complex movements
- ✅ **Safety limits** and value clamping
- ✅ **Mock components** for Gazebo plugin independence
- ✅ **Comprehensive testing** script

> **Note**: Zoom plugin is currently disabled to avoid errors. Focus on pan/tilt controllers.