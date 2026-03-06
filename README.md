# ptz_gz_sim

This package provides a Gazebo-based simulation for the PTZ (Pan-Tilt-Zoom) camera system using **Gazebo Garden**. The package has been tested on ROS 2 Humble with Gazebo Garden.

The `launch_sim.launch` file can be used to start the entire simulation stack in a ROS 2 workspace.

## Prerequisites

1. A working ROS 2 installation (Humble recommended).
2. Gazebo Garden and the ROS 2 Gazebo plugins (`ros-humble-ros-gz-sim`, `ros-humble-ros-gz-bridge`) installed.
3. ROS 2 control packages for managing controllers, e.g. `ros-humble-ros2-control` and `ros-humble-ros2-controllers`.
4. Ensure the workspace is built and sourced.

## Launching the simulation

Once the workspace is built and sourced, start the Gazebo Garden simulation using the provided launch file:

```bash
# from anywhere in the system after sourcing your workspace
ros2 launch ptz_gz_sim launch_sim.launch
```

The command will:

1. Start Gazebo Garden with the `leonardo_race.sdf` world found in `worlds/`
2. Spawn the PTZ camera robot description (`axis_camera_gazebo.xacro`) and publish its TF via `robot_state_publisher`.
3. Launch the `ros_gz_bridge` to connect ROS 2 topics with Gazebo Garden messages (clock, images, camera info, zoom commands, etc.).
4. Start the controller manager along with several ros2_control plugins:
   - `joint_state_broadcaster` for reporting joint states.
   - `position_controller` and `velocity_controller` for the pan/tilt joints.
   These are spawned via the `controller_manager` nodes and configured by the URDF's `libgz_ros2_control` tags along with parameters from `config/controller_params.yaml`.
5. A `fake_axis_camera_node` is launched to mimic the axis camera interface.
6. RViz2 is started with a default configuration from the `ptz_manager` package so you can visualize the camera and TF frames.

## Migration from Gazebo Fortress to Garden

This package has been migrated from Gazebo Fortress to **Gazebo Garden**. Key changes include:

- Updated package dependencies from `ignition-*` to `gz-*` packages
- Updated CMake configuration to use `gz-cmake4` instead of `ignition-cmake2`
- Plugin registration uses `GZ_ADD_PLUGIN` macro instead of `IGNITION_ADD_PLUGIN`
- SDF materials have been updated to use Gazebo Garden-compatible format
- ROS 2 integration uses `ros_gz_sim` and `ros_gz_bridge` packages

