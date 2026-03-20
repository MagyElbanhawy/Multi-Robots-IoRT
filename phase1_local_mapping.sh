#!/bin/bash
# Phase 1: Decentralized Local Mapping (EMRMF)

source ~/emrmf_ws/install/setup.bash

echo "Starting Phase 1: Decentralized Local Mapping"

# 1. Launch EAI T-mini Pro LiDAR
echo "Launching LiDAR..."
ros2 launch ydlidar_ros2_driver ydlidar_launch.py &

# 2. Launch Orbbec DaBai Camera
echo "Launching Camera..."
ros2 launch orbbec_camera ob_camera.launch.py depth_registration:=true &

# 3. Setup Static Transforms
echo "Publishing Static Transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser_frame &
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link &

# 4. Launch RTAB-Map
echo "Launching RTAB-Map..."
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/depth/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    scan_topic:=/scan \
    approx_sync:=true \
    frame_id:=base_link &

# 5. Launch RViz
echo "Launching RViz..."
ros2 run rviz2 rviz2 &

echo "Phase 1 started successfully. Press Ctrl+C to terminate all processes."

# Wait for all background processes to finish
wait
