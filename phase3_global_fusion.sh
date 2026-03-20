#!/bin/bash
# Phase 3: Multi-Robot Global Map Fusion (EMRMF Central Server)

source ~/emrmf_ws/install/setup.bash

echo "Starting Phase 3: Multi-Robot Global Map Fusion"

# 1. Global Fusion Server Node (Placeholder)
echo "Starting Global Map Fusion Node..."
ros2 run emrmf_core global_map_fusion_node &

# 2. Dynamic Task Allocation Node (Placeholder)
echo "Starting Task Allocation Node..."
ros2 run emrmf_core dynamic_task_allocation_node &

echo "Phase 3 started successfully. Press Ctrl+C to terminate all processes."

# Wait for all background processes to finish
wait
