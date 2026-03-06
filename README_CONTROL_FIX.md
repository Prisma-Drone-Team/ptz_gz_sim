# PTZ Camera Simulation - ROS2 Control

## Description

Resolved issues with ROS Control and zoom plugins following the `dtt_hyrman_description` approach:

### Changes implemented:

1. **ros2_control separation**: Created separate `urdf/ptz.ros2_control.xacro` file
2. **mock_components usage**: Replaced `gz_ros2_control/GazeboSimSystem` with `mock_components/GenericSystem`
3. **Gazebo plugin removal**: Removed `gz_ros2_control::GazeboSimROS2ControlPlugin` plugin
4. **Standalone controller manager**: Controller management through separate nodes
5. **Zoom plugin fix**: Fixed CameraZoomPlugin name and syntax

### Files created/modified:

- `urdf/ptz.ros2_control.xacro` - Separate ros2_control configuration
- `config/ptz_controllers.yaml` - PTZ controller configuration
- `launch/ptz_controllers.launch.py` - Complete launch file with fake camera
- `launch/ptz_test.launch.py` - Test launch file for controllers only
- `urdf/axis_camera_gazebo.xacro` - Modified to use mock_components
- `CMakeLists.txt` - Added controller dependencies
- `package.xml` - Added controller dependencies

## How to test:

### Controllers only test (without Gazebo):
```bash
ros2 launch ptz_gz_sim ptz_test.launch.py
```

### Complete test with fake camera:
```bash
ros2 launch ptz_gz_sim ptz_controllers.launch.py
```

### Manual joint control:
```bash
# Publish joint commands
ros2 topic pub /ptz_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3]"

# View joint state
ros2 topic echo /joint_states
```

## Advantages of new configuration:

- ✅ Controllers work without Gazebo plugin errors
- ✅ Clear separation between simulation and control
- ✅ Fixed zoom plugin
- ✅ Easily testable without Gazebo
- ✅ Modular and maintainable structure