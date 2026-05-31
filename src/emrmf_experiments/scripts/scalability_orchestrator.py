#!/usr/bin/env python3

import os
import time
import subprocess
import argparse

robot_counts = [2, 3, 4, 5]
runs_per_config = 5
run_duration_sec = 60 # Simulated time per run for scalability
output_dir = '/tmp/emrmf_experiments'

def main():
    parser = argparse.ArgumentParser(description="Run EMRMF scalability experiments")
    parser.add_argument('--dry-run', action='store_true', help='Only print commands')
    args = parser.parse_args()

    print("Starting EMRMF Scalability Orchestrator...")
    os.makedirs(output_dir, exist_ok=True)

    total_runs = len(robot_counts) * runs_per_config
    current_run = 0

    print(f"Total robot count configurations: {len(robot_counts)}")
    print(f"Total runs (x{runs_per_config}): {total_runs}")

    for count in robot_counts:
        for i in range(runs_per_config):
            current_run += 1
            run_id = f"scale_robots_{count}_run{i}"

            print(f"[{current_run}/{total_runs}] Starting: {run_id} with {count} robots")

            # Construct the launch command for the Gazebo simulation and logger
            cmd = [
                "ros2", "launch", "emrmf_experiments", "multi_robot_sim.launch.py",
                f"robot_count:={count}",
                f"lidar_noise:=0.05"
            ]

            logger_cmd = [
                "ros2", "run", "emrmf_experiments", "experiment_logger_node",
                "--ros-args",
                "-p", "ablation_mode:=full_emrmf",
                "-p", f"robot_count:={count}",
                "-p", f"run_id:={run_id}",
                "-p", f"output_dir:={output_dir}"
            ]

            driver_cmd = [
                "python3", "src/emrmf_experiments/scripts/trajectory_driver.py",
                "--robot_count", str(count)
            ]

            if args.dry_run:
                print(" ".join(cmd))
                print(" ".join(logger_cmd))
                print(" ".join(driver_cmd))
                continue

            # In real execution, we would start Gazebo, the Driver, and the Logger
            sim_proc = subprocess.Popen(cmd)
            time.sleep(10) # wait for Gazebo to load models

            logger_proc = subprocess.Popen(logger_cmd)
            driver_proc = subprocess.Popen(driver_cmd)

            try:
                # Wait for duration
                time.sleep(run_duration_sec)

                # Signal experiment done
                subprocess.run(["ros2", "topic", "pub", "--once", "/experiment_done", "std_msgs/msg/Bool", "{data: true}"], stderr=subprocess.DEVNULL)
                logger_proc.wait(timeout=10)

            except subprocess.TimeoutExpired:
                print(f"Time limit reached ({run_duration_sec}s). Terminating run.")

            # Clean up
            sim_proc.terminate()
            logger_proc.terminate()
            driver_proc.terminate()

            try:
                sim_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim_proc.kill()
                driver_proc.kill()

            subprocess.run(["pkill", "-f", "experiment_logger_node"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "trajectory_driver.py"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "gzserver"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "gzclient"], stderr=subprocess.DEVNULL)

            time.sleep(5) # Cooldown between heavily taxing gazebo spawns

    print("Scalability Orchestration complete! Use report_generator.py to generate tables.")

if __name__ == '__main__':
    main()
