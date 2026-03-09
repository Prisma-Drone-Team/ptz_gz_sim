# PTZ Navigation Stack Integration

## Overview

This setup integrates the hardware PTZ motion stack (ptz_manager) with the Gazebo simulation, enabling:
- Hardware-compatible PTZ control in simulation
- SEED command processing (`cover`, `watchto`)
- YOLO-based ROI detection
- QR code scanning with ZBar
- Seamless topic remapping between hardware and simulation APIs

## Architecture

### Simulation Layer
- **PTZ Gazebo model**: Simulated camera with pan/tilt/zoom
- **ROS2 Control**: Mock components with position/velocity controllers  
- **fake_axis_camera**: Action server mimicking hardware axis camera interface
- **Camera bridge**: Links Gazebo camera to ROS2 topics

### Hardware Motion Stack  
- **ptz_manager**: High-level PTZ controller (SEED commands → PTZ actions)
- **Direct compatibility**: Uses `/cmd/velocity` topic and `move_ptz/position_abs` action
- **Topic remapping**: Only `/axis/state` → `/ptz_state` needed
- **No bridge required**: ptz_manager directly uses simulation-compatible APIs

### Integration Layer
- **Minimal remapping**: Hardware topics → simulation topics  
- **Direct action calls**: ptz_manager uses `move_ptz/position_abs` action client
- **Supporting stack**: YOLO, ZBar, image republisher

## Usage

### 1. Launch Simulation (Terminal 1)
```bash
# Launch PTZ in rover world simulation
ros2 launch ptz_gz_sim spawn_ptz_in_rover_world.launch.py
```

### 2. Launch Navigation Stack (Terminal 2)  
```bash
# Launch PTZ motion stack with simulation integration
ros2 launch ptz_gz_sim ptz_navigation_stack.launch.py

# Optional parameters:
ros2 launch ptz_gz_sim ptz_navigation_stack.launch.py use_rviz:=true use_yolo:=false use_zbar:=true
```

### 3. Test Commands

#### Manual PTZ Control
```bash
# Manual position command
ros2 topic pub --once cmd/ptz ptz_action_server_msgs/msg/Ptz "{pan: 0.5, tilt: 0.3, zoom: 5.0}"

# Velocity control  
ros2 topic pub --once /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.1, tilt: 0.05, zoom: 0.0}"

# Stop velocity
ros2 topic pub --once /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.0, tilt: 0.0, zoom: 0.0}"
```

#### SEED Commands
```bash
# Coverage task command
ros2 topic pub --once /seed_pdt_camera/command std_msgs/msg/String "data: 'cover(null,34,2,0:13:00)'"

# Coverage with specific zone
ros2 topic pub --once /seed_pdt_camera/command std_msgs/msg/String "data: 'cover((1,0),(5,0),(5,3),(1,3),34,2,0:13:00)'"

# Watch specific TF frame  
ros2 topic pub --once /seed_pdt_camera/command std_msgs/msg/String "data: 'watchto(target_frame)'"
```

## Topics Overview

### Simulation Topics
- `/ptz_state` - Current PTZ state (position, mode)
- `/joint_states` - Joint positions and velocities
- `/camera/image_raw` - Camera image stream
- `move_ptz/position_abs` - Position action server

### Hardware Stack Topics  
- `/cmd/velocity` - Velocity commands
- `/seed_pdt_camera/command` - SEED high-level commands
- `cmd/ptz` - Manual PTZ commands
- `/yolo/detections` - YOLO object detections
- `/barcode` - QR code detections

### Remapped Topics
- `/axis/state` → `/ptz_state` (hardware feedback → simulation state)
- `/cmd/velocity` → **Direct compatibility** (no remapping needed)  
- `move_ptz/position_abs` → **Direct compatibility** (action server)

## Configuration

### PTZ Manager Parameters
Edit `src/ptz_manager/param/param.yaml`:
- `camera_tf_name`: Camera frame name
- `cover_vel_ctrl_*`: Velocity control gains  
- `arena_corner_points_from_map`: Coverage area bounds
- Topic names and rates

### Simulation Parameters
Edit `src/ptz_gz_sim/config/ptz_controllers.yaml`:
- Controller update rates
- Joint limits and names

## Troubleshooting

### Velocity Control Not Working
```bash
# Check if velocity timer is running
ros2 topic echo /rosout | grep "Velocity mode"

# Verify position controller is active
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Check velocity integration logs
ros2 topic echo /position_controller/commands
```

### SEED Commands Not Working
```bash
# Check ptz_manager is receiving commands
ros2 topic echo /rosout | grep -i "cover\|watchto"

# Verify action server connection
ros2 action list
ros2 action info move_ptz/position_abs
```

### Camera Stream Issues
```bash
# Check camera topics
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw

# Test image republisher
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/image_raw/compressed --remap out:=/image_raw
```

## Architecture Diagram

```
┌─────────────────┐    ┌───────────────────┐    ┌──────────────────────┐
│   SEED Commands │───▶│   ptz_manager     │───▶│   Gazebo Simulation  │
│   cover(...)    │    │   - Coverage      │    │   - PTZ Model        │
│   watchto(...)  │    │   - Velocity ctrl │    │   - ROS2 Control     │
└─────────────────┘    │   - TF tracking   │    │   - Camera Bridge    │
                       └───────────────────┘    └──────────────────────┘
┌─────────────────┐             │                          ▲
│   YOLO/ZBar     │             │ Topic Remapping          │  
│   - ROI detect  │             ▼                          │
│   - QR codes    │    ┌───────────────────┐               │
└─────────────────┘───▶│  Topic Bridge     │───────────────┘
                       │  /axis/state →    │
                       │  /ptz_state       │
                       └───────────────────┘
```