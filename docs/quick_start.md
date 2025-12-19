# Quick Start - Everything is Ready!

## ✅ Status Check

Your system is configured and ready:
- ✅ **LiDAR**: Connected and reachable at 192.168.1.62
- ✅ **Camera**: Hardware detected, drivers installed
- ✅ **Network**: eth0 configured on 192.168.1.100/24
- ✅ **ROS 2**: Installed and ready

## Launch Everything

Simply run:

```bash
cd ~/glitter
./launch_fusion.sh
```

This will automatically:
1. Start LiDAR driver → `/unilidar/cloud`
2. Start Camera driver → `/camera/image_raw`
3. Start Fusion node → `/unilidar/colored_cloud`
4. Launch RViz → Visualize colored point cloud

## What You'll See

In RViz, you should see:
- **Colored point cloud** combining LiDAR 3D points with camera colors
- **Real-time updates** as the LiDAR scans
- **RGB colors** from the camera mapped onto the 3D points

## If Something Doesn't Work

1. **Check logs**: Look at terminal output for errors
2. **Verify topics**: 
   ```bash
   ros2 topic list
   ros2 topic hz /unilidar/cloud
   ros2 topic hz /camera/image_raw
   ros2 topic hz /unilidar/colored_cloud
   ```
3. **Run diagnostic**:
   ```bash
   ./scripts/utilities/diagnose_system.sh
   ```

## Manual Launch (Alternative)

If you prefer to launch components separately:

**Terminal 1 - LiDAR:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Terminal 2 - Camera:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run usb_cam usb_cam_node
```

**Terminal 3 - Fusion:**
```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source activate_env.sh
python3 src/core/fusion.py
```

**Terminal 4 - RViz:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/glitter/config/l2_fusion.rviz
```

---

**You're all set! Run `./launch_fusion.sh` to see your colored point clouds!** 🎉










