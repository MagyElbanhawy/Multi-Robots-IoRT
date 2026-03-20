#!/bin/bash
# Phase 2: Map Collecting Using the Trust Factor (EMRMF)

source ~/emrmf_ws/install/setup.bash

echo "Starting Phase 2: Map Collecting Using the Trust Factor"

# 1. IMU and Odometry Integration (Optional but recommended)
echo "Launching IMU and Odometry Integration..."
ros2 launch robot_localization ekf.launch.py &

# 2. Trust Factor Node (Placeholder)
echo "Starting Trust Factor Node..."
# ros2 run emrmf_core trust_factor_node &

# 3. LoRa Communication Node (Placeholder)
echo "Starting LoRa Communication Node..."
# ros2 run emrmf_comms lora_transmitter_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 &

echo "Phase 2 started successfully. Press Ctrl+C to terminate all processes."

# Wait for all background processes to finish
wait
