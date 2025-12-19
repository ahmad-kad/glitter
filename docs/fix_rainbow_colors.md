# Fix: Seeing Rainbow Colors Instead of Camera RGB

## Problem
You're seeing rainbow/distance-colored point clouds instead of actual camera RGB colors.

## Cause
RViz is displaying the **raw LiDAR** point cloud (with intensity/distance coloring) instead of the **fused colored cloud** from the camera.

## Solution

### Step 1: Check Which Display is Active in RViz

In RViz, look at the **Displays** panel on the left:

1. **Raw_L2_LiDAR** (or similar) - This shows rainbow/intensity colors
   - **Disable this** by unchecking the Enabled checkbox

2. **Fused_PointCloud** - This should show camera RGB colors
   - **Enable this** by checking the Enabled checkbox
   - **Topic**: Should be `/unilidar/colored_cloud`
   - **Color Transformer**: Should be `RGB8`

### Step 2: Verify Fusion is Running

Check if the fusion node is actually publishing colored clouds:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check if colored cloud topic exists
ros2 topic list | grep colored

# Check if it's publishing
ros2 topic hz /unilidar/colored_cloud
```

**If the topic doesn't exist or isn't publishing:**
- The fusion node might not be running
- Camera might not be publishing images
- Fusion might be failing silently

### Step 3: Verify Camera is Working

The fusion needs camera images to color the points:

```bash
# Check camera topic
ros2 topic hz /camera/image_raw

# View camera image
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**If camera isn't publishing:**
- Camera driver might not be running
- Camera might not be connected
- Check camera logs

### Step 4: Check RViz Display Settings

In RViz, for the **Fused_PointCloud** display:

1. **Enabled**: ✓ Checked
2. **Topic**: `/unilidar/colored_cloud`
3. **Color Transformer**: `RGB8` (NOT "Intensity" or "Z")
4. **Size (Pixels)**: 5-10 for visibility
5. **Fixed Frame**: `unilidar_lidar`

### Step 5: Restart Everything

If still not working, restart the system:

```bash
# Stop everything (Ctrl+C in launch terminal)

# Restart
cd ~/glitter
./launch_fusion.sh
```

Then in RViz:
- Disable **Raw_L2_LiDAR** display
- Enable **Fused_PointCloud** display
- Set Color Transformer to **RGB8**

## Quick Checklist

- [ ] **Fused_PointCloud** display is **Enabled**
- [ ] **Raw_L2_LiDAR** display is **Disabled**
- [ ] Topic = `/unilidar/colored_cloud`
- [ ] Color Transformer = `RGB8`
- [ ] Fusion node is running
- [ ] Camera is publishing `/camera/image_raw`
- [ ] Colored cloud topic exists: `ros2 topic list | grep colored`

## Why Rainbow Colors?

Rainbow colors come from:
- **Intensity coloring**: LiDAR intensity values mapped to colors
- **Distance coloring**: Z-axis (depth) values mapped to rainbow colormap
- **Default RViz behavior**: When no RGB data is available, RViz uses intensity/distance

## Expected Result

After fixing:
- ✅ Points colored with actual camera RGB values
- ✅ Colors match what the camera sees
- ✅ Real-time color updates as camera captures new images
- ✅ No rainbow/distance-based coloring

## Troubleshooting

### Still seeing rainbow colors?

1. **Check fusion node logs**: Look at terminal where fusion is running
2. **Verify camera alignment**: Colors might not match if camera-LiDAR aren't calibrated
3. **Check point cloud fields**: 
   ```bash
   ros2 topic echo /unilidar/colored_cloud --once | grep -A 10 "fields"
   ```
   Should show an `rgb` field

### Colors don't match camera view?

This is a **calibration issue**. The camera and LiDAR need to be aligned:

```bash
cd ~/glitter
source activate_env.sh
python3 src/core/calibration.py
```

Use the calibration tool to adjust camera-LiDAR alignment until colors match the 3D points correctly.
