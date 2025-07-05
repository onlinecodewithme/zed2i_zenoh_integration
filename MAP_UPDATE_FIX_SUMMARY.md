# Map Update Issue Fix Summary

## Problem Description
The map was creating initially but not updating when the camera moved. This is a common issue in SLAM systems where the robot appears stationary to the mapping algorithm.

## Root Cause
The issue was caused by a **static transform** from `odom` to `base_link` that was overriding the ZED camera's dynamic odometry. This made the SLAM system think the robot never moved, so it wouldn't update the map.

## Solution Applied

### 1. Removed Static Odometry Transform
- **Removed**: Static transform publisher for `odom` -> `base_link`
- **Reason**: This was preventing the ZED's visual odometry from working properly

### 2. Enhanced ZED Odometry Configuration
The ZED camera launch is configured with:
```yaml
publish_odom_tf: 'true'    # Enable ZED odometry TF publishing
odom_frame: 'odom'         # Use standard odom frame
base_frame: 'base_link'    # Use standard base_link frame
```

### 3. Optimized SLAM Parameters
- **Reduced `minimum_time_interval`**: From 0.5s to 0.1s for more responsive mapping
- **Increased `transform_timeout`**: From 0.2s to 0.5s for better transform handling

### 4. Enlarged Local Costmap
- **Increased size**: From 3x3m to 6x6m to accommodate sensor positioning
- **Added `always_send_full_costmap: true`**: For better costmap updates

## How It Works Now

### Transform Chain
```
map -> odom -> base_link -> zed2i_base_link -> camera_frames
     ↑        ↑
   SLAM    ZED Visual
 Toolbox   Odometry
```

### Dynamic Mapping Process
1. **ZED Visual Odometry**: Tracks camera movement and publishes `odom` -> `base_link` transform
2. **SLAM Toolbox**: Receives odometry and laser scans, builds map, publishes `map` -> `odom` transform
3. **Map Updates**: As camera moves, new areas are explored and added to the map
4. **Loop Closure**: When returning to previously mapped areas, the map is optimized

## Expected Behavior Now

✅ **Map creates initially** - SLAM starts building map from first scan
✅ **Map updates with movement** - New areas appear as camera moves
✅ **Dynamic expansion** - Map grows organically without size limits
✅ **Loop closure** - Map corrects itself when revisiting areas
✅ **Real-time updates** - Faster response to camera movement (0.1s intervals)

## Testing the Fix

1. **Launch the system**: `./run_nav2_slam.sh`
2. **Wait for initialization**: Let all nodes start and initial map appear
3. **Move the camera**: Slowly move the ZED camera around
4. **Observe in RViz**: You should see:
   - Map expanding as you explore new areas
   - Robot pose updating in real-time
   - New obstacles appearing as you encounter them

## Troubleshooting

If map still doesn't update:

1. **Check TF tree**: `ros2 run tf2_tools view_frames`
   - Should show: `map -> odom -> base_link -> camera_frames`

2. **Check odometry**: `ros2 topic echo /zed2i/zed_node/odom`
   - Should show changing position values as camera moves

3. **Check scan data**: `ros2 topic echo /scan`
   - Should show laser scan data updating

4. **Monitor SLAM**: Look for SLAM Toolbox messages about pose updates

## Key Configuration Files Modified

- `src/zed_occupancy_mapping/launch/zed_nav2_slam.launch.py`
- `src/zed_occupancy_mapping/config/slam_params.yaml`
- `src/zed_occupancy_mapping/config/nav2_params.yaml`

The system now properly uses ZED's visual odometry for dynamic mapping!
