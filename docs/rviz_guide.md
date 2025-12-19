# RViz Visualization Guide

Complete guide for visualizing LiDAR point clouds in RViz, including setup, navigation, troubleshooting, and advanced features.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Navigation Controls](#navigation-controls)
3. [Configuration](#configuration)
4. [Point Cloud Persistence](#point-cloud-persistence)
5. [Troubleshooting](#troubleshooting)
6. [Alternative Visualization Methods](#alternative-visualization-methods)

---

## Quick Start

### Launch LiDAR and RViz

```bash
cd ~/glitter
./launch_lidar_rviz.sh
```

Or launch separately:

**Terminal 1 - LiDAR Driver:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Terminal 2 - RViz:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/glitter/config/l2_fusion.rviz
```

### Initial Configuration

1. **Enable Point Cloud Display:**
   - Left panel: **Displays** → **Raw_L2_LiDAR**
   - **✓ Check the Enabled checkbox**

2. **Verify Topic:**
   - **Raw_L2_LiDAR** → **Topic**: `/unilidar/cloud`

3. **Set Fixed Frame:**
   - **Global Options** → **Fixed Frame**: `unilidar_lidar`

4. **Adjust Point Size:**
   - **Raw_L2_LiDAR** → **Size (Pixels)**: `5` or `10` (for better visibility)

5. **Reset View:**
   - **Views** → **Current View** → **Reset** button
   - Or press **R** key

---

## Navigation Controls

### Mouse Controls (Default)

- **Left Click + Drag**: Rotate camera around the scene
- **Middle Click + Drag**: Pan/move view left/right/up/down
- **Right Click + Drag**: Zoom in/out
- **Scroll Wheel**: Zoom in/out

### Keyboard Shortcuts

- **R**: Reset view to default
- **F**: Focus on selected object
- **Q**: Quit RViz
- **I**: Interact tool (default - for rotating/panning)
- **M**: Move tool
- **P**: Place/Pose tool

### View Types

Change view type in **Views** → **Current View** → Dropdown:

- **Orbit** (default): Rotate around focal point - good for inspecting point clouds
- **FPS (First Person)**: Move like a game
  - **W/A/S/D**: Move forward/left/backward/right
  - **Q/E**: Move up/down
  - **Mouse**: Look around
- **Top Down Orthographic**: View from above, no perspective distortion
- **XY Orthographic**: Side view

### Adjust View Settings

**Views** → **Current View** → Expand:
- **Distance**: How far camera is from focal point
- **Focal Point**: Center point of rotation [x, y, z]
- **Yaw**: Horizontal rotation (left/right)
- **Pitch**: Vertical rotation (up/down)

---

## Configuration

### RViz Settings Summary

| Setting | Location | Recommended Value |
|---------|----------|-------------------|
| **Fixed Frame** | Global Options | `unilidar_lidar` |
| **Topic** | Raw_L2_LiDAR | `/unilidar/cloud` |
| **Decay Time** | Raw_L2_LiDAR | `1000.0` (for persistence) |
| **Size (Pixels)** | Raw_L2_LiDAR | `5-10` |
| **Color Transformer** | Raw_L2_LiDAR | `Intensity` |
| **Enabled** | Raw_L2_LiDAR | ✓ Checked |

### Point Cloud Display Options

- **Size (Pixels)**: Point size in pixels (2-20 recommended)
- **Color Transformer**: 
  - `Intensity`: Color by LiDAR intensity (recommended)
  - `FlatColor`: Single color
  - `RGB`: If RGB data available
- **Decay Time**: How long points persist (0 = real-time only, 1000+ = accumulate)
- **Queue Size**: Buffer size (5-10 recommended)

---

## Point Cloud Persistence

### Method 1: RViz Decay Time (Visual Only)

**In RViz:**
1. **Displays** → **Raw_L2_LiDAR** → **Decay Time**
2. Set to:
   - `1000.0` = Keep points for ~16 minutes
   - `3600.0` = Keep points for 1 hour
   - `999999.0` = Keep forever (until RViz closes)

**Limitations:**
- Only visual - doesn't save to file
- Lost when RViz closes
- Can slow down RViz with too many points

### Method 2: Accumulator Node (Saves to File)

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Basic usage
python3 scripts/utilities/accumulate_pointcloud.py

# With options
python3 scripts/utilities/accumulate_pointcloud.py \
  --decay-time 300 \
  --max-points 2000000 \
  --output my_room.pcd
```

**Options:**
- `--decay-time 300`: Keep points for 5 minutes (default: 60s)
- `--max-points 2000000`: Max 2 million points (default: 1M)
- `--voxel-size 0.05`: Downsample to 5cm voxels (default: 0.05m)
- `--output room.pcd`: Output filename

**View Accumulated Cloud in RViz:**
1. **Displays** → **Add** → **PointCloud2**
2. **Name**: `Accumulated_Room`
3. **Topic**: `/unilidar/accumulated_cloud`
4. **Fixed Frame**: `unilidar_lidar`

---

## Troubleshooting

### Problem: RViz Crashes (Exit Code -11)

**Causes:**
- Complex RViz config with too many displays
- Memory issues from large point clouds
- OpenGL/driver issues
- Config corruption

**Solutions:**

1. **Use Simple Config (Recommended):**
   ```bash
   rviz2 -d ~/glitter/config/l2_fusion_simple.rviz
   ```

2. **Reduce Decay Time:**
   - **Displays** → **Raw_L2_LiDAR** → **Decay Time**: Change from `1000.0` to `100.0` or `50.0`

3. **Disable Unused Displays:**
   - Disable **TF_Frames** (if causing issues)
   - Disable **Fused_PointCloud** (if not using fusion)
   - Disable **Raw_Camera_Image** (if no camera)

4. **Reduce Queue Size:**
   - **Displays** → **Raw_L2_LiDAR** → **Queue Size**: Change from `10` to `5` or `3`

5. **Launch Separately:**
   - Launch LiDAR and RViz in separate terminals
   - If RViz crashes, LiDAR keeps running

6. **Check System Resources:**
   ```bash
   free -h  # Check memory
   ```

### Problem: Point Cloud Not Visible

**Checklist:**
- [ ] Display is **Enabled** (checkbox checked)
- [ ] Topic = `/unilidar/cloud`
- [ ] Fixed Frame = `unilidar_lidar`
- [ ] Size (Pixels) = 5 or higher
- [ ] Data is publishing (check with `ros2 topic hz /unilidar/cloud`)

**Diagnostic Commands:**
```bash
# Check if data is flowing
ros2 topic hz /unilidar/cloud

# Check subscription
ros2 topic info /unilidar/cloud -v | grep Subscription
# Should show: Subscription count: 1 or more

# Verify point cloud data
./scripts/utilities/verify_pointcloud.sh
```

**Solutions:**
1. **Increase Point Size**: Try `10` or `20` pixels
2. **Reset View**: **Views** → **Current View** → **Reset**
3. **Check Data**: Run `ros2 topic echo /unilidar/cloud --once`
4. **Create New Display**: If existing display doesn't work, add a new PointCloud2 display

### Problem: "No tf data" or "frame base_link does not exist"

**Cause:** RViz is trying to use a TF frame that doesn't exist.

**Solution 1: Set Fixed Frame Correctly (Recommended)**
- **Global Options** → **Fixed Frame**: `unilidar_lidar` (not `base_link` or `map`)
- PointCloud2 works without TF if Fixed Frame matches message frame_id

**Solution 2: Create Static TF Transform**
```bash
# Launch static TF broadcaster
./scripts/utilities/launch_static_tf.sh

# Or manually
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map unilidar_lidar
```

**Solution 3: Disable TF Display**
- **Displays** → **TF_Frames** → Uncheck to disable (optional)

### Problem: "PointCloud2 could not transform to base_link"

**Cause:** RViz Fixed Frame is set to `base_link` but no transform exists.

**Solution:**
1. **Global Options** → **Fixed Frame**: Change to `unilidar_lidar`
2. Or create the transform (see above)

### Problem: RViz Not Subscribed to Topic

**Symptoms:** Subscription count = 0

**Solution:**
1. **Enable Display**: Check the Enabled checkbox
2. **Verify Topic**: Ensure topic = `/unilidar/cloud`
3. **Restart RViz**: Close completely and relaunch
4. **Create New Display**: If existing display doesn't work

### Problem: Points Disappear Immediately

**Cause:** Decay Time is set to `0.0`

**Solution:**
- **Displays** → **Raw_L2_LiDAR** → **Decay Time**: Change to `1000.0` or higher

### Problem: Camera Stuck or View Issues

**Solutions:**
- Press **R** to reset view
- **Views** → **Current View** → **Reset**
- Adjust **Distance** to `5.0` or `10.0`
- Check if point cloud is visible (display enabled?)

---

## Alternative Visualization Methods

### Method 1: Python Open3D Viewer

If RViz isn't working, use Python viewer:

```bash
source venv/bin/activate
pip install open3d
python3 scripts/utilities/visualize_with_open3d.py
```

### Method 2: Command Line Viewer (Quick Check)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 scripts/utilities/test_pointcloud_viewer.py
```

Shows point count, X/Y/Z ranges, and sample points.

### Method 3: Save and View Point Cloud File

```bash
# Save one frame
python3 scripts/utilities/save_pointcloud.py snapshot.pcd

# View with external tools
meshlab snapshot.pcd
# or
cloudcompare snapshot.pcd
```

---

## Best Practices

1. **Start Simple**: Use minimal config for testing
2. **Monitor Memory**: Watch system memory with large point clouds
3. **Use Decay Time Wisely**: Balance between persistence and performance
4. **Separate Launches**: Launch LiDAR and RViz separately for stability
5. **Save Configs**: Save working RViz configs for reuse
6. **Regular Saves**: Use accumulator node to save point clouds regularly

---

## Quick Reference

```
┌─────────────────────────────────┐
│ RViz Navigation Controls        │
├─────────────────────────────────┤
│ Left Click + Drag  → Rotate     │
│ Middle Click + Drag → Pan        │
│ Right Click + Drag  → Zoom       │
│ Scroll Wheel        → Zoom       │
│ R Key               → Reset      │
│ F Key               → Focus      │
│ W/A/S/D (FPS mode)  → Move       │
└─────────────────────────────────┘
```

**Recommended Settings:**
- Fixed Frame: `unilidar_lidar`
- Topic: `/unilidar/cloud`
- Decay Time: `1000.0`
- Size: `5-10` pixels
- Color: Intensity











