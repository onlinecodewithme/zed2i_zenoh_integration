# ZED2i Navigation and SLAM with Nav2

This ROS2 project integrates the ZED2i stereo camera with Nav2 navigation stack and SLAM Toolbox for autonomous navigation and real-time mapping.

## Overview

The project consists of:
- **ZED ROS2 Wrapper**: Official Stereolabs wrapper for ZED cameras
- **Nav2 Navigation Stack**: Complete autonomous navigation solution
- **SLAM Toolbox**: Real-time SLAM for mapping and localization
- **Point Cloud to Laser Scan**: Converts ZED point cloud to 2D laser scan for SLAM
- **Launch Files**: Automated startup for the complete navigation system
- **RViz Configuration**: Pre-configured visualization for navigation and mapping

## Features

- Real-time SLAM using ZED2i depth data
- Autonomous navigation with path planning and obstacle avoidance
- Dynamic map building and localization
- Point cloud to laser scan conversion for SLAM compatibility
- Integrated visualization with RViz2 and Nav2 tools
- Support for both live camera and SVO file playback
- Goal setting and waypoint navigation through RViz

## Prerequisites

### Hardware
- ZED2i stereo camera
- NVIDIA GPU (recommended for optimal performance)
- USB 3.0 port
- Robot platform with odometry (optional but recommended)

### Software
- Ubuntu 22.04 LTS
- ROS2 Humble
- ZED SDK 4.0+
- CUDA 11.2+ (for GPU acceleration)

### ROS2 Dependencies
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-pointcloud-to-laserscan
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-tf2-tools
```

## Installation

1. **Install ZED SDK**
   ```bash
   # Download and install ZED SDK from https://www.stereolabs.com/developers/release/
   # Follow the installation instructions for your system
   ```

2. **Clone and Build the Workspace**
   ```bash
   # Navigate to your workspace
   cd /path/to/zed_integration
   
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Install dependencies
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build the workspace
   colcon build --symlink-install
   
   # Source the workspace
   source install/setup.bash
   ```

## Usage

### Basic Launch (SLAM + Navigation)
```bash
# Source the workspace
source install/setup.bash

# Launch the complete Nav2 SLAM system
ros2 launch zed_occupancy_mapping zed_nav2_slam.launch.py
```

### Launch with Custom Parameters
```bash
# Launch with specific parameters
ros2 launch zed_occupancy_mapping zed_nav2_slam.launch.py \
    slam_params_file:=/path/to/custom_slam_params.yaml \
    nav2_params_file:=/path/to/custom_nav2_params.yaml
```

## Navigation Usage

### Setting Navigation Goals

1. **Using RViz2 Navigation Panel:**
   - Open RViz2 (launched automatically)
   - Use the "Nav2 Goal" tool to set navigation targets
   - Click on the map to set goal positions

2. **Using Command Line:**
   ```bash
   # Set a navigation goal
   ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
   {
     header: {stamp: {sec: 0}, frame_id: 'map'},
     pose: {
       position: {x: 2.0, y: 1.0, z: 0.0},
       orientation: {w: 1.0}
     }
   }"
   ```

3. **Saving Maps:**
   ```bash
   # Save the current map
   ros2 run nav2_map_server map_saver_cli -f ~/my_map
   ```

## Configuration

### SLAM Parameters

Key SLAM Toolbox parameters in `config/slam_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Map resolution (meters per pixel)
    resolution: 0.05
    
    # Maximum laser range for mapping
    max_laser_range: 8.0
    
    # Loop closure settings
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
```

### Navigation Parameters

Key Nav2 parameters in `config/nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    # Controller frequency
    controller_frequency: 20.0
    
    # Robot tolerances
    xy_goal_tolerance: 0.25
    yaw_goal_tolerance: 0.25

local_costmap:
  local_costmap:
    ros__parameters:
      # Local costmap size (meters)
      width: 3
      height: 3
      resolution: 0.05
```

### Point Cloud to Laser Scan Configuration

The system converts ZED point cloud to laser scan with these parameters:

- **Height range**: -0.5m to 2.0m
- **Angle range**: -90° to +90°
- **Range**: 0.45m to 8.0m
- **Angular resolution**: 0.5°

## Topics

### Published Topics
- `/map` (nav_msgs/OccupancyGrid): SLAM-generated map
- `/scan` (sensor_msgs/LaserScan): Converted laser scan from point cloud
- `/plan` (nav_msgs/Path): Planned navigation path
- `/local_costmap/costmap` (nav_msgs/OccupancyGrid): Local obstacle costmap
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid): Global navigation costmap

### Subscribed Topics
- `/zed2i/zed_node/point_cloud/cloud_registered` (sensor_msgs/PointCloud2): ZED point cloud
- `/goal_pose` (geometry_msgs/PoseStamped): Navigation goals
- `/odom` (nav_msgs/Odometry): Robot odometry (if available)

## Coordinate Frames

- `map`: Global SLAM map frame
- `odom`: Odometry frame (provided by SLAM when no robot odometry available)
- `base_link`: Robot base frame
- `zed2i_base_link`: Camera base frame
- `zed2i_left_camera_frame`: Left camera frame (used for laser scan)

## Troubleshooting

### Common Issues

1. **ZED Camera Not Detected**
   ```bash
   # Check if camera is connected
   lsusb | grep ZED
   
   # Test ZED SDK installation
   /usr/local/zed/tools/ZED_Explorer
   ```

2. **No Laser Scan Data**
   ```bash
   # Check point cloud topic
   ros2 topic echo /zed2i/zed_node/point_cloud/cloud_registered
   
   # Check laser scan conversion
   ros2 topic echo /scan
   ```

3. **SLAM Not Working**
   - Ensure adequate lighting conditions
   - Check that laser scan data is being published
   - Verify transform tree: `ros2 run tf2_tools view_frames`

4. **Navigation Issues**
   - Check costmap topics for obstacle detection
   - Verify goal poses are reachable
   - Adjust navigation parameters for your robot

### Performance Optimization

- Use GPU acceleration with CUDA
- Adjust SLAM frequency for your hardware
- Optimize costmap update rates
- Reduce point cloud density if needed

## Advanced Usage

### Custom Robot Integration

To integrate with your own robot:

1. **Replace Robot Description:**
   - Modify the robot URDF in the launch file
   - Update transform from `base_link` to `zed2i_base_link`

2. **Add Robot Odometry:**
   - Publish odometry on `/odom` topic
   - Update SLAM parameters to use robot odometry

3. **Customize Navigation:**
   - Adjust robot footprint and radius
   - Tune controller parameters for your robot dynamics

### Multi-Robot Support

The system can be extended for multi-robot scenarios by:
- Using namespaces in launch files
- Configuring separate map frames
- Setting up robot-specific parameters

## Development

### Code Structure

```
src/zed_occupancy_mapping/
├── launch/
│   └── zed_nav2_slam.launch.py     # Main launch file
├── config/
│   ├── slam_params.yaml            # SLAM Toolbox configuration
│   ├── nav2_params.yaml            # Nav2 navigation parameters
│   └── nav2_slam.rviz              # RViz configuration
├── CMakeLists.txt                  # Build configuration
└── package.xml                     # Package metadata
```

### Extending Functionality

The system can be extended with:
- **Custom Nav2 Plugins**: Add specialized planners or controllers
- **Advanced SLAM**: Integrate other SLAM algorithms
- **Sensor Fusion**: Combine multiple sensors
- **Behavior Trees**: Create custom navigation behaviors

## License

This project is provided as-is for educational and research purposes. Please refer to the ZED SDK license for commercial use restrictions.

## Support

For issues related to:
- **ZED Camera/SDK**: Visit [Stereolabs Support](https://support.stereolabs.com/)
- **Nav2**: Check [Nav2 Documentation](https://navigation.ros.org/)
- **SLAM Toolbox**: See [SLAM Toolbox Repository](https://github.com/SteveMacenski/slam_toolbox)
- **ROS2**: Check [ROS2 Documentation](https://docs.ros.org/en/humble/)

## References

- [Nav2 Navigation Stack](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Point Cloud to Laser Scan](https://github.com/ros-perception/pointcloud_to_laserscan)
