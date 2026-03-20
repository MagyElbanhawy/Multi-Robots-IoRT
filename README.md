# EMRMF Single Robot Setup and Launch Instructions

This repository contains the setup script and launch instructions to run a single robot based on the EMRMF (Multi-Robot Graph SLAM-based Trust Factor) framework.
The setup is targeted for Ubuntu 22.04 LTS and ROS 2 Humble.

## Setup Instructions

1. Ensure you have Ubuntu 22.04 installed.
2. Run the `setup_single_robot.sh` script to install ROS 2 Humble, hardware drivers (EAI T-mini Pro, Orbbec DaBai), SLAM dependencies (RTAB-Map, g2o), and other required tools.

   ```bash
   chmod +x setup_single_robot.sh
   ./setup_single_robot.sh
   ```
3. Source your environment:

   ```bash
   source ~/.bashrc
   ```

## Launch Instructions

To launch the system, you will need to open multiple terminals. Be sure to run `source ~/emrmf_ws/install/setup.bash` in each new terminal.

### Phase 1: Decentralized Local Mapping

This phase initializes the hardware sensors and sets up the local coordinate frames.

**Terminal 1: Launch EAI T-mini Pro LiDAR**
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**Terminal 2: Launch Orbbec DaBai Camera**
```bash
ros2 launch orbbec_camera ob_camera.launch.py depth_registration:=true
```

**Terminal 3: Setup Static Transforms**
Establish the coordinate frame transformations between the robot base and its sensors.
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser_frame
# In the same terminal or a new one, run:
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link
```

**Terminal 4: Launch RTAB-Map**
Start the decentralized local mapping process using RTAB-Map.
```bash
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/depth/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    scan_topic:=/scan \
    approx_sync:=true \
    frame_id:=base_link
```

**Terminal 5: Launch RViz for visualization**
```bash
ros2 run rviz2 rviz2
```

### Phase 2: Map Collecting Using the Trust Factor ($\theta$)

This phase introduces the trust factor to mitigate the effects of communication delays and sensor noise on pose estimation.

*Note: The actual implementation of the dynamic trust factor $\theta$ will require a custom ROS 2 node or an extension to the existing RTAB-Map pose graph optimizer. Below are placeholder commands for running the trust factor node and LoRa communication.*

**Terminal 6: Launch IMU and Odometry Integration (Optional but recommended)**
```bash
ros2 launch robot_localization ekf.launch.py
```

**Terminal 7: Launch Trust Factor Node (Placeholder)**
Start the custom node that calculates the trust factor $\theta$ based on pre- and post-optimization poses.
```bash
# This is a placeholder for the custom EMRMF trust factor node.
# ros2 run emrmf_core trust_factor_node
```

**Terminal 8: Launch LoRa Communication Node (Placeholder)**
Start the node responsible for transmitting local maps and poses to the central server via LoRa.
```bash
# This is a placeholder for the custom EMRMF LoRa node.
# ros2 run emrmf_comms lora_transmitter_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```