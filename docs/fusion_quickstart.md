# Quick Start: Colored Point Clouds

Step-by-step guide to see colored point clouds from LiDAR + Camera fusion.

## Prerequisites

✅ **LiDAR**: Running and publishing to `/unilidar/cloud`  
✅ **Camera**: Running and publishing to `/camera/image_raw`  
✅ **ROS 2**: Sourced and working

## Step-by-Step Instructions

### Step 1: Launch LiDAR Driver

**Terminal 1:**
```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch LiDAR driver
ros2 launch unitree_lidar_ros2 launch.py
```

**Verify LiDAR is working:**
```bash
# In another terminal, check topics
ros2 topic list | grep unilidar
# Should see: /unilidar/cloud

# Check data rate
ros2 topic hz /unilidar/cloud
# Should show ~10-20 Hz
```

### Step 2: Launch Camera Driver

**Terminal 2:**

**For Raspberry Pi Camera:**
```bash
source /opt/ros/jazzy/setup.bash

# Using picamera2 (if you have a ROS 2 bridge)
# Or use your camera driver that publishes to /camera/image_raw
ros2 run your_camera_package camera_node
```

**For USB Camera:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run usb_cam usb_cam_node
```

**Verify Camera is working:**
```bash
# Check camera topic
ros2 topic list | grep camera
# Should see: /camera/image_raw

# Check camera data rate
ros2 topic hz /camera/image_raw
# Should show ~15-30 Hz
```

### Step 3: Run Fusion Node

**Terminal 3:**
```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Activate Python environment (if using venv)
source activate_env.sh  # or: source venv/bin/activate

# Run fusion node
python3 src/core/fusion.py
```

**What to expect:**
- Fusion node will start and wait for LiDAR + Camera data
- You should see log messages about synchronization
- Once both sensors are publishing, fusion will begin
- Colored point cloud will be published to `/unilidar/colored_cloud`

**Verify Fusion is working:**
```bash
# Check colored cloud topic
ros2 topic list | grep colored
# Should see: /unilidar/colored_cloud

# Check fusion rate
ros2 topic hz /unilidar/colored_cloud
# Should show ~10-20 Hz (matches LiDAR rate)
```

### Step 4: Visualize in RViz

**Terminal 4:**
```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch RViz
rviz2 -d config/l2_fusion.rviz
```

**In RViz:**

1. **Enable Colored Point Cloud Display:**
   - **Displays** → Find **Fused_PointCloud** or **Colored_Cloud**
   - **✓ Check Enabled checkbox**
   - **Topic**: Should be `/unilidar/colored_cloud`

2. **Set Fixed Frame:**
   - **Global Options** → **Fixed Frame**: `unilidar_lidar`

3. **Adjust Settings:**
   - **Size (Pixels)**: `5` or `10` for better visibility
   - **Color Transformer**: `RGB8` (to see colors from camera)

4. **Reset View:**
   - **Views** → **Current View** → **Reset**
   - Use mouse to rotate/pan/zoom

**You should now see:**
- ✅ 3D point cloud colored with camera data
- ✅ Real-time updates as LiDAR scans
- ✅ Colors matching what the camera sees

## Troubleshooting

### Problem: No Colored Cloud Topic

**Check fusion node logs:**
- Look at Terminal 3 where fusion is running
- Should see messages about receiving LiDAR/Camera data

**Verify both sensors are publishing:**
```bash
# Check LiDAR
ros2 topic hz /unilidar/cloud

# Check Camera
ros2 topic hz /camera/image_raw
```

**Solution:** Make sure both LiDAR and Camera are running and publishing data.

### Problem: Colored Cloud Has No Colors (All White/Gray)

**Possible causes:**
1. **Camera not aligned with LiDAR** - Need calibration
2. **Wrong color transformer** - Set to `RGB8` in RViz
3. **Camera image is black/blank** - Check camera is working

**Check camera image:**
```bash
# View camera image
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**Solution:** 
- Verify camera is capturing images
- Check if calibration is needed (see Calibration section)

### Problem: Fusion Rate is Low

**Check synchronization:**
- Fusion node uses time synchronization
- If sensors have very different rates, fusion may be slow

**Solution:**
- Ensure both sensors are publishing at reasonable rates (10+ Hz)
- Check network/system load

### Problem: Points Not Visible in RViz

**Checklist:**
- [ ] Display is **Enabled**
- [ ] Topic = `/unilidar/colored_cloud`
- [ ] Fixed Frame = `unilidar_lidar`
- [ ] Size (Pixels) = 5 or higher
- [ ] Color Transformer = `RGB8`

See [RViz Guide](rviz_guide.md) for detailed troubleshooting.

## Using Launch Script (Alternative)

If you have a launch file that starts everything:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch everything at once (if you have a launch file)
ros2 launch your_package fusion.launch.py
```

## Calibration (If Colors Don't Match)

If the colors don't align properly with the 3D points, you may need calibration:

```bash
cd ~/glitter
source activate_env.sh
python3 src/core/calibration.py
```

This will help you adjust the camera-LiDAR alignment.

## Quick Reference

**Topics:**
- `/unilidar/cloud` - Raw LiDAR point cloud
- `/camera/image_raw` - Camera images
- `/unilidar/colored_cloud` - **Colored point cloud (output)**

**Commands:**
```bash
# Terminal 1: LiDAR
ros2 launch unitree_lidar_ros2 launch.py

# Terminal 2: Camera
ros2 run usb_cam usb_cam_node  # or your camera driver

# Terminal 3: Fusion
python3 src/core/fusion.py

# Terminal 4: RViz
rviz2 -d config/l2_fusion.rviz
```

**RViz Settings:**
- Fixed Frame: `unilidar_lidar`
- Topic: `/unilidar/colored_cloud`
- Color Transformer: `RGB8`
- Size: `5-10` pixels

---

**Next Steps:**
- See [Troubleshooting Guide](troubleshooting.md) for more help
- See [RViz Guide](rviz_guide.md) for visualization tips
- See [Hardware Setup](../HARDWARE_SETUP.md) for camera setup details











