# Orbbec LiDAR ROS2 Driver

This ROS2 driver enables you to interface with the **Orbbec Single Line LiDAR** using the [Orbbec LiDAR SDK](https://github.com/orbbec/orbbec_lidar_sdk), wrapped within a ROS2-compatible environment. This document provides installation instructions, usage guidelines, and other essential information to help you get started with the driver.

## 1. Installation

### 1.1. Prerequisites

Before using the Orbbec LiDAR ROS2 driver, ensure the following dependencies are installed on your system:

- **ROS2**: A valid installation of ROS2 (Jazzy, Humble, Foxy, or another supported distribution).
  - If you need assistance, refer to the [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html).

### 1.2. Clone the Repository

Clone the Orbbec LiDAR ROS2 driver repository from GitHub:

```bash
git clone https://github.com/orbbec/orbbec_lidar_ros2.git
```

### 1.3. Build the Package

After cloning the repository, build the package using `colcon`:

```bash
cd orbbec_lidar_ros2
colcon build --symlink-install
```

This will compile the driver and create symbolic links to the build files. The `--symlink-install` flag allows you to develop without the need to rebuild after making changes.

## 2. Usage

### 2.1. Running the Driver

To start the driver, launch the provided ROS2 launch file:

```bash
source install/setup.bash
# Launch the driver with point cloud data
ros2 launch orbbec_lidar_ros2 single_line.launch.py data_type:=pointcloud
# Or launch the driver with laser scan data
ros2 launch orbbec_lidar_ros2 single_line.launch.py data_type:=laserscan
```

This command will initiate the node that interfaces with the Orbbec Single Line LiDAR device. Ensure your LiDAR hardware is properly connected before running the command.

### 2.2. Visualizing the Point Cloud

After running the driver, you can visualize the LiDAR data in **RViz2**, a 3D visualization tool for ROS. Follow these steps to visualize the point cloud data:

1. Open RViz2:

   ```bash
   rviz2
   ```
2. In RViz2, add a **PointCloud2** or **LaserScan** topic to visualize the incoming data:

   - Set fixed frame to the frame ID used by the driver (e.g., `scan`).
   - Click on **Add** in the RViz2 interface.
   - Choose **By Topic** and select the point cloud topic (e.g., `/scan` or `/point_cloud`).
3. The LiDAR data should now be visible in the RViz2 window as point cloud data or a laser scan.

### 2.3. Parameters and Configuration

The `single_line.launch.py` file contains default parameters for the driver. You can customize these settings by modifying the launch file or creating a custom configuration file. Key parameters include:

- **lidar_name**: The name of the LiDAR sensor, used to differentiate between multiple LiDAR devices. Default: `'lidar'`.
- **type**: The specific type of the LiDAR sensor. Defaults to `'single_line'`.
- **frame_id**: The reference frame in which the LiDAR data will be published (e.g., `"scan"`). Default: `'scan'`.
- **data_type**: Specifies the type of data published by the LiDAR. Options include `"laserscan"` or `"pointcloud"`. Default: `'laserscan'`.
- **data_qos**: The Quality of Service (QoS) settings for the data topic, affecting reliability and delivery guarantees. Default: `'sensor_data'`.
- **use_intra_process**: A boolean indicating whether intra-process communication is used, which can reduce latency. Default: `'false'`.
- **min_angle**: The minimum angle of the LiDAR’s scanning range, in degrees (e.g., `-135.0`). Default: `'-135.0'`.
- **max_angle**: The maximum angle of the LiDAR’s scanning range, in degrees (e.g., `135.0`). Default: `'135.0'`.
- **min_range**: The minimum measurable distance by the LiDAR, in meters. Default: `'0.05'`.
- **max_range**: The maximum measurable distance by the LiDAR, in meters. Default: `'30.0'`.
- **scan_frequency**: The scan data emission rate, in Hertz (Hz). Default: `'30.0'`.
- **config_file**: The path to a configuration file (typically in TOML format) containing additional device-specific settings.
- **shared_container_name**: The name of the shared ROS2 component container. Default: `'orbbec_lidar_container'`.
- **attach_to_shared_container**: A boolean flag indicating whether the node should be attached to an existing shared container. Default: `'false'`.
- **use_hardware_time**: A boolean indicating whether the driver use hardware timestamps. Default: `'false'`.
- **filter_level**: Define the LiDAR filter level. Default: `0`.
- **enable_smoothing_filter**: A boolean indicating whether the driver use smoothing filter. Default: `'false'`.

### 2.4. Retrieving Device IP and Port

To retrieve the device's IP address and port, run the following command in your terminal:

```bash
ros2 run orbbec_lidar_ros2 list_devices_node
```

This command will list the connected LiDAR devices and display their respective IP addresses and ports. You can use this information to configure the driver to connect to a specific device.

### 2.5. Config File

The configuration file is located at `orbbec_lidar_ros2/config/single_line_lidar.toml`. You can modify the IP address and port in this file to connect to a different device.

### 2.6. Setting Device IP Address

If you need to change the IP address of your Orbbec LiDAR device, you can use the `set_device_ip` tool. This is useful when setting up a new device or reconfiguring your network.

#### Step 1: List Available Devices

First, discover the connected devices to find the current IP address:

```bash
source install/setup.bash
ros2 run orbbec_lidar_ros2 list_devices_node
```

This command will scan for devices and display their IP addresses and ports. Wait a few seconds for the scan to complete. Example output:

```
New device connected: 192.168.1.121:2228
```

#### Step 2: Set the Device IP Address

Use the current IP address from Step 1 to set a new IP address for the device:

```bash
ros2 run orbbec_lidar_ros2 set_device_ip_node <current_ip> <new_ip> [port]
```

Example (using the IP from Step 1):
```bash
ros2 run orbbec_lidar_ros2 set_device_ip_node 192.168.1.121 192.168.1.200
```

Or with a custom port:
```bash
ros2 run orbbec_lidar_ros2 set_device_ip_node 192.168.1.121 192.168.1.200 2228
```

**Note:** You must always specify the current IP address. Use `list_devices_node` (from Step 1) to find the current IP if you don't know it.

#### Step 3: Update Configuration File

After successfully changing the device IP address, update the configuration file to use the new IP:

1. Edit the config file:
   ```bash
   nano orbbec_lidar_ros2/config/single_line_lidar.toml
   ```

2. Update the IP address in the file:
   ```toml
   ip = "192.168.1.200"  # Your new IP address
   port = 2228
   ```

#### Important Notes

- **Device Reboot**: The device may need to be rebooted for the new IP address to take effect. Power cycle the device after changing the IP.

- **Network Configuration**: Ensure the new IP address is on the same subnet as your computer and does not conflict with other devices on your network.

- **Multiple Devices**: If multiple devices are detected when listing, note each device's IP address and specify the correct current IP when setting the new IP.

- **Persistence**: The tool automatically saves the configuration to the device's flash memory, so the new IP will persist after reboot.

- **Verification**: After setting the IP and rebooting the device, you can verify the new IP using:
  ```bash
  ros2 run orbbec_lidar_ros2 verify_device_ip_node <new_ip>
  ```

## 3. Troubleshooting

- **No point cloud data in RViz**: Ensure the topic is correctly added in RViz, check that the driver is running without errors, and verify that your LiDAR hardware is functional and powered on.
- **Build issues**: Make sure all dependencies, particularly `libpcl-dev` and ROS2, are properly installed. Missing dependencies can cause build failures.

## 4. Additional Resources

- [Orbbec LiDAR SDK](https://github.com/orbbec/orbbec_lidar_sdk): The official SDK for interfacing with Orbbec LiDAR devices.
- [ROS2 Documentation](https://docs.ros.org/en/): Official documentation for ROS2, including installation guides and tutorials.
- [Point Cloud Library (PCL)](http://pointclouds.org/): Learn more about PCL, an essential library for working with 3D data.

## 5. License

This project is licensed under the **Apache License 2.0**. You may obtain a copy of the license at:

```
http://www.apache.org/licenses/LICENSE-2.0
```
