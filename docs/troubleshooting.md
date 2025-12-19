# Troubleshooting Guide

Common issues and solutions for LiDAR-Camera Fusion system.

## Table of Contents

1. [Point Cloud Issues](#point-cloud-issues)
2. [RViz Issues](#rviz-issues)
3. [TF Transform Issues](#tf-transform-issues)
4. [Network Issues](#network-issues)
5. [Performance Issues](#performance-issues)
6. [General Diagnostics](#general-diagnostics)

---

## Point Cloud Issues

### Problem: No Point Cloud Data

**Symptoms:**
- Topic not publishing
- `ros2 topic list` doesn't show `/unilidar/cloud`
- Empty point cloud messages

**Diagnostics:**
```bash
# Check if topic exists
ros2 topic list | grep unilidar

# Check topic info
ros2 topic info /unilidar/cloud

# Check if data is flowing
ros2 topic hz /unilidar/cloud

# Check point cloud data
ros2 topic echo /unilidar/cloud --once
```

**Solutions:**

1. **Check LiDAR Connection:**
   ```bash
   # Ping LiDAR
   ping 192.168.1.62  # or 192.168.1.150
   
   # Check network interface
   ip addr show eth0
   ```

2. **Verify Driver is Running:**
   ```bash
   # Check if driver node is running
   ros2 node list | grep lidar
   
   # Check driver logs
   # Look at terminal where driver was launched
   ```

3. **Restart LiDAR Driver:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch unitree_lidar_ros2 launch.py
   ```

4. **Check Driver Installation:**
   ```bash
   # Verify driver is built
   ls ~/ros2_ws/install/unitree_lidar_ros2/lib/unitree_lidar_ros2/
   
   # Rebuild if needed
   cd ~/ros2_ws
   colcon build --packages-select unitree_lidar_ros2
   source install/setup.bash
   ```

### Problem: Low Point Cloud Rate

**Symptoms:**
- Point cloud publishing at < 5 Hz
- Stuttering or delayed updates

**Diagnostics:**
```bash
# Check actual rate
ros2 topic hz /unilidar/cloud

# Check network bandwidth
iftop -i eth0  # Install: sudo apt install iftop
```

**Solutions:**

1. **Check Network Configuration:**
   ```bash
   # Verify network speed
   ethtool eth0 | grep Speed
   
   # Check for packet loss
   ping -c 100 192.168.1.62 | grep loss
   ```

2. **Adjust LiDAR Settings:**
   - Edit launch file to reduce `cloud_scan_num`
   - Reduce `range_max` if not needed
   - Check LiDAR firmware version

3. **Check System Load:**
   ```bash
   # Monitor CPU usage
   htop
   
   # Check for other processes using network
   sudo netstat -tulpn | grep :2368
   ```

### Problem: Invalid Point Cloud Data

**Symptoms:**
- Points all at origin (0,0,0)
- NaN or inf values in point cloud
- Wrong frame_id

**Diagnostics:**
```bash
# Check point cloud structure
ros2 topic echo /unilidar/cloud --once | head -50

# Use Python to inspect
python3 scripts/utilities/test_pointcloud_viewer.py
```

**Solutions:**

1. **Check Frame ID:**
   ```bash
   # Verify frame_id in messages
   ros2 topic echo /unilidar/cloud --once | grep frame_id
   
   # Should be: unilidar_lidar
   ```

2. **Verify LiDAR Calibration:**
   - Check if LiDAR needs calibration
   - Verify LiDAR firmware is up to date

3. **Check Driver Configuration:**
   - Review launch file parameters
   - Verify IP address is correct

---

## RViz Issues

### Problem: RViz Crashes (Segmentation Fault)

**Symptoms:**
- RViz exits with code -11
- Crashes when loading config
- Crashes when displaying point cloud

**Solutions:**

1. **Use Simple Config:**
   ```bash
   rviz2 -d ~/glitter/config/l2_fusion_simple.rviz
   ```

2. **Reduce Memory Usage:**
   - **Decay Time**: Reduce from 1000.0 to 100.0
   - **Queue Size**: Reduce from 10 to 5
   - Disable unused displays

3. **Check System Resources:**
   ```bash
   # Check memory
   free -h
   
   # Check if system is low on memory
   # Close other applications if < 500MB free
   ```

4. **Launch Separately:**
   - Launch LiDAR in one terminal
   - Launch RViz in another terminal
   - If RViz crashes, LiDAR keeps running

5. **Check OpenGL:**
   ```bash
   glxinfo | grep "OpenGL version"
   # Should show OpenGL 3.0+
   ```

See [RViz Guide](rviz_guide.md) for detailed troubleshooting.

### Problem: Point Cloud Not Visible in RViz

**Symptoms:**
- RViz running but no points visible
- Subscription count = 0

**Checklist:**
- [ ] Display is **Enabled** (checkbox checked)
- [ ] Topic = `/unilidar/cloud`
- [ ] Fixed Frame = `unilidar_lidar`
- [ ] Size (Pixels) = 5 or higher
- [ ] Data is publishing (check with `ros2 topic hz`)

**Solutions:**

1. **Enable Display:**
   - **Displays** → **Raw_L2_LiDAR** → **✓ Check Enabled**

2. **Verify Topic:**
   - **Raw_L2_LiDAR** → **Topic**: `/unilidar/cloud`

3. **Set Fixed Frame:**
   - **Global Options** → **Fixed Frame**: `unilidar_lidar`

4. **Increase Point Size:**
   - **Raw_L2_LiDAR** → **Size (Pixels)**: `10` or `20`

5. **Reset View:**
   - **Views** → **Current View** → **Reset**

6. **Check Subscription:**
   ```bash
   ros2 topic info /unilidar/cloud -v | grep Subscription
   # Should show: Subscription count: 1 or more
   ```

---

## TF Transform Issues

### Problem: "No tf data" or "frame base_link does not exist"

**Symptoms:**
- RViz shows TF errors
- Point cloud not displaying due to TF issues

**Solutions:**

1. **Set Fixed Frame Correctly (Recommended):**
   - **Global Options** → **Fixed Frame**: `unilidar_lidar`
   - PointCloud2 works without TF if Fixed Frame matches message frame_id

2. **Create Static TF Transform:**
   ```bash
   # Launch static TF broadcaster
   ./scripts/utilities/launch_static_tf.sh
   
   # Or manually
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map unilidar_lidar
   ```

3. **Disable TF Display:**
   - **Displays** → **TF_Frames** → Uncheck to disable

### Problem: "PointCloud2 could not transform to base_link"

**Symptoms:**
- RViz error about transform failure
- Point cloud not displaying

**Cause:** RViz Fixed Frame is set to `base_link` but no transform exists.

**Solution:**
1. **Global Options** → **Fixed Frame**: Change to `unilidar_lidar`
2. Or create the transform (see above)

**Verify:**
```bash
# Check TF chain exists
ros2 run tf2_ros tf2_echo map unilidar_lidar

# Or list all frames
ros2 run tf2_ros tf2_monitor
```

### Understanding PointCloud vs PointCloud2

**Key Points:**

- **Topic Name** = Where data is published (can be anything)
  - `/unilidar/cloud` ← This is just a name
  - `/pointcloud` ← Could be this name
  - `/pointcloud2` ← Could be this name

- **Message Type** = What format the data is in
  - `sensor_msgs/msg/PointCloud2` ← This is the actual format (modern)
  - `sensor_msgs/msg/PointCloud` ← Older format (deprecated)

**Your LiDAR publishes:**
- ✅ **Topic name**: `/unilidar/cloud` (custom name chosen by driver)
- ✅ **Message type**: `sensor_msgs/msg/PointCloud2` (correct format)
- ✅ **RViz can display it**: Use "PointCloud2" display type

**Check message type:**
```bash
ros2 topic info /unilidar/cloud
# Shows: Type: sensor_msgs/msg/PointCloud2
```

---

## Network Issues

### Problem: Cannot Reach LiDAR

**Symptoms:**
- `ping 192.168.1.62` fails
- Driver cannot connect to LiDAR

**Diagnostics:**
```bash
# Check network interface
ip addr show eth0

# Check routing
ip route show

# Check ARP table
ip neigh show

# Test connectivity
ping -c 3 192.168.1.62
```

**Solutions:**

1. **Check Network Configuration:**
   ```bash
   # Verify IP address
   ip addr show eth0
   # Should show: 192.168.1.100/24 (or similar)
   
   # Check if interface is up
   ip link show eth0
   # Should show: state UP
   ```

2. **Reset Network Connection:**
   ```bash
   # Restart network interface
   sudo nmcli connection down "l2-lidar"
   sudo nmcli connection up "l2-lidar"
   
   # Or restart networking
   sudo systemctl restart NetworkManager
   ```

3. **Check Firewall:**
   ```bash
   # Check firewall status
   sudo ufw status
   
   # Allow LiDAR network
   sudo ufw allow from 192.168.1.0/24
   ```

4. **Verify LiDAR IP:**
   - Default: `192.168.1.62` or `192.168.1.150`
   - Check LiDAR documentation for actual IP
   - May need to configure LiDAR IP via web interface

5. **Check Cable:**
   - Verify Ethernet cable is connected
   - Try different cable
   - Check cable for damage

### Problem: High Network Latency

**Symptoms:**
- Delayed point cloud updates
- High latency in data

**Diagnostics:**
```bash
# Check latency
ping -c 10 192.168.1.62 | grep "avg"

# Check network speed
ethtool eth0 | grep Speed

# Monitor network traffic
iftop -i eth0
```

**Solutions:**

1. **Check Network Speed:**
   - Should be 100 Mbps or 1 Gbps
   - Verify cable supports required speed

2. **Reduce Network Load:**
   - Close other network applications
   - Disable unnecessary services

3. **Check for Interference:**
   - Ensure cable is not damaged
   - Keep cable away from power sources

---

## Performance Issues

### Problem: High CPU Usage

**Symptoms:**
- System slow or unresponsive
- CPU usage > 90%

**Diagnostics:**
```bash
# Check CPU usage
htop

# Check temperature
vcgencmd measure_temp

# Check which processes are using CPU
ps aux --sort=-%cpu | head -10
```

**Solutions:**

1. **Enable Performance Governor:**
   ```bash
   sudo cpupower frequency-set -g performance
   ```

2. **Reduce Point Cloud Size:**
   - Reduce `range_max` in LiDAR launch file
   - Reduce `cloud_scan_num`
   - Use voxel downsampling

3. **Close Unnecessary Processes:**
   - Close other applications
   - Disable unnecessary services

4. **Check Thermal Throttling:**
   ```bash
   # Check temperature
   vcgencmd measure_temp
   # Should be < 80°C
   
   # If overheating, add cooling or reduce load
   ```

### Problem: High Memory Usage

**Symptoms:**
- System running out of memory
- Processes killed by OOM killer

**Diagnostics:**
```bash
# Check memory usage
free -h

# Check which processes use memory
ps aux --sort=-%mem | head -10
```

**Solutions:**

1. **Reduce RViz Decay Time:**
   - Lower Decay Time in RViz
   - Don't accumulate too many points

2. **Use Voxel Downsampling:**
   ```bash
   python3 scripts/utilities/accumulate_pointcloud.py --voxel-size 0.1
   ```

3. **Enable Swap:**
   ```bash
   # Create swap file
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

4. **Close Unnecessary Applications:**
   - Close other applications
   - Restart RViz periodically

### Problem: Low Fusion Rate

**Symptoms:**
- Fusion running at < 10 FPS
- Stuttering or delayed processing

**Diagnostics:**
```bash
# Check fusion node performance
# Look at fusion node logs for timing information

# Check system load
htop

# Check memory
free -h
```

**Solutions:**

1. **Optimize Point Cloud Size:**
   - Reduce point cloud range
   - Use voxel filtering

2. **Check Camera Performance:**
   - Verify camera is publishing at expected rate
   - Check camera resolution

3. **Profile Fusion Code:**
   ```bash
   python3 -m cProfile -o profile.stats src/core/fusion.py
   ```

---

## General Diagnostics

### Comprehensive System Check

```bash
# Run full diagnostic
./scripts/test_hardware.sh

# Check ROS topics
ros2 topic list
ros2 topic hz /unilidar/cloud

# Check ROS nodes
ros2 node list

# Check TF frames
ros2 run tf2_ros tf2_monitor

# Check system resources
free -h
df -h
vcgencmd measure_temp
```

### Verify Point Cloud Data

```bash
# Quick verification script
./scripts/utilities/verify_pointcloud.sh

# Or manually check
ros2 topic info /unilidar/cloud -v
ros2 topic echo /unilidar/cloud --once
ros2 topic hz /unilidar/cloud
```

### Check Driver Installation

```bash
# Verify driver is built
ls ~/ros2_ws/install/unitree_lidar_ros2/lib/unitree_lidar_ros2/

# Check if package is available
ros2 pkg list | grep unitree

# Rebuild if needed
cd ~/ros2_ws
colcon build --packages-select unitree_lidar_ros2
source install/setup.bash
```

### Check ROS Environment

```bash
# Verify ROS is sourced
echo $ROS_DISTRO
# Should show: jazzy

# Check ROS installation
ros2 --version

# List available packages
ros2 pkg list | head -20
```

---

## Getting Help

If issues persist:

1. **Check Logs:**
   - Driver logs (terminal where driver was launched)
   - RViz logs (terminal where RViz was launched)
   - System logs: `journalctl -n 50`

2. **Run Diagnostics:**
   ```bash
   ./scripts/test_hardware.sh
   ./scripts/utilities/verify_pointcloud.sh
   ```

3. **Review Documentation:**
   - [RViz Guide](rviz_guide.md)
   - [Hardware Setup](../HARDWARE_SETUP.md)
   - [Development Guide](../DEVELOPMENT_GUIDE.md)

4. **Collect Information:**
   - ROS version: `ros2 --version`
   - System info: `uname -a`
   - Network config: `ip addr show`
   - Topic info: `ros2 topic info /unilidar/cloud`











