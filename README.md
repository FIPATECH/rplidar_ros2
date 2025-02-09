# RPLIDAR ROS2 package

ROS2 node and test application for RPLIDAR

***Note:*** I just made some changes to update this package for ROS2 compatibility, modifying the original `rplidar_ros` repository for use with my RPLIDAR A2M8 on a Jetson Nano.

For more details about RPLIDAR, please visit:

- **rplidar roswiki:** [http://wiki.ros.org/rplidar](http://wiki.ros.org/rplidar)
- **RPLIDAR HomePage:** [http://www.slamtec.com/en/Lidar](https://www.slamtec.com/en/Lidar/A2)
- **RPLIDAR SDK:** [https://github.com/Slamtec/rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
- **RPLIDAR Tutorial:** [https://github.com/robopeak/rplidar_ros/wiki](https://github.com/robopeak/rplidar_ros/wiki)

## How to build the package

1. **Clone the repository**

   Clone this project into the `src` folder of your colcon workspace:

   ```bash
   cd [your-ros-package-directory]/src
   git clone https://github.com/FIPATECH/rplidar_ros2.git
   ```

2. **Install ROS2 and Colcon**

   Make sure you have ROS2 (installed with the necessary build tools (colcon, etc.).

3. **Build the package**

   ```bash
   cd [your-ros-package-directory]
   colcon build --symlink-install
   source ./install/setup.bash
   ```

4. **Verify the package exists**

   ```bash
   ros2 pkg list | grep rplidar_ros2
   ```

## Running the node and visualizing in RViz

To launch the RPLIDAR node along with RViz, run:

```bash
ros2 launch rplidar_ros2 view_rplidar_launch.py
```

***Notice:*** The default parameters are set for RPLIDAR A2M8 with a **115200** baud rate. If using a different model, adjust the baud rate and other relevant parameters in the launch files accordingly.

## RPLidar frame  
The RPLidar frame should be broadcasted as shown in *rplidar_AX.png*.
