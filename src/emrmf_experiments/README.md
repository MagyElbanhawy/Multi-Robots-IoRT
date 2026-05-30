# EMRMF Experiments

This package provides an automated orchestrator, network proxy, and logger to generate reviewer-ready experimental evidence for the EMRMF Multi-Robot SLAM framework.

## Features
- **Network Proxy Node**: Simulates artificial delay and packet loss on ROS 2 topics.
- **Experiment Logger Node**: Calculates Pose RMSE, Map Alignment RMSE, logs trust factor variables, and saves output to CSV.
- **Orchestrator**: Automates running various parameter configurations (ablation modes, p, gamma, delay, loss) for statistical robustness (5 runs per config).
- **Report Generator**: Parses raw output and generates CSV, Markdown, and LaTeX tables including Mean, Std, and 95% Confidence Intervals.

## How to use

1. Build the workspace:
   ```bash
   cd ~/emrmf_ws
   colcon build --packages-select emrmf_experiments
   source install/setup.bash
   ```

2. Run the orchestrator:
   ```bash
   ros2 run emrmf_experiments orchestrator.py
   ```
   *Note: Ensure your ground truth provider and other necessary simulated/hardware nodes are running or integrated into the launch file.*

3. Generate Journal Tables:
   ```bash
   ros2 run emrmf_experiments report_generator.py
   ```

4. View Results:
   Results are located in `/tmp/emrmf_experiments/` by default.
