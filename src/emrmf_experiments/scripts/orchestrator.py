#!/usr/bin/env python3

import os
import time
import subprocess
import argparse

# Configuration Space
p_values = [2, 3, 4]
gamma_values = [0.1, 0.3, 0.5, 1.0]
delay_values = [0.0, 0.5, 2.0]
packet_loss_values = [0.0, 0.10, 0.30]
ablation_modes = ["baseline_graph_slam", "decentralized_only", "trust_only", "full_emrmf"]
runs_per_config = 5

run_duration_sec = 120
output_dir = '/tmp/emrmf_experiments'

def main():
    parser = argparse.ArgumentParser(description="Run EMRMF experiments")
    parser.add_argument('--dry-run', action='store_true', help='Only print commands')
    args = parser.parse_args()

    print("Starting EMRMF Orchestrator...")
    os.makedirs(output_dir, exist_ok=True)

    configs_to_run = []

    # 1. Ablation Study (No delay, no loss, fixed p=2, gamma=0.1)
    for mode in ablation_modes:
        configs_to_run.append((mode, 2.0, 0.1, 0.0, 0.0))

    # 2. Trust Factor Sensitivity (full_emrmf, no delay/loss)
    for p in p_values:
        for gamma in gamma_values:
            if (p != 2.0 or gamma != 0.1): # Avoid duplicate with ablation
                configs_to_run.append(("full_emrmf", float(p), float(gamma), 0.0, 0.0))

    # 3. Communication Robustness (full_emrmf and baseline, fixed p=2, gamma=0.1, varying delay/loss)
    for mode in ["baseline_graph_slam", "full_emrmf"]:
        for d in delay_values:
            for l in packet_loss_values:
                if d != 0.0 or l != 0.0:
                    configs_to_run.append((mode, 2.0, 0.1, float(d), float(l)))

    total_runs = len(configs_to_run) * runs_per_config
    current_run = 0

    print(f"Total configurations: {len(configs_to_run)}")
    print(f"Total runs (x{runs_per_config}): {total_runs}")

    for config in configs_to_run:
        mode, p, gamma, delay, loss = config
        for i in range(runs_per_config):
            current_run += 1
            run_id = f"{mode}_p{p}_g{gamma}_d{delay}_l{loss}_run{i}"

            print(f"[{current_run}/{total_runs}] Starting: {run_id}")

            # Construct the launch command
            cmd = [
                "ros2", "launch", "emrmf_experiments", "experiment.launch.py",
                f"ablation_mode:={mode}",
                f"p:={p}",
                f"gamma:={gamma}",
                f"delay_sec:={delay}",
                f"packet_loss_rate:={loss}",
                f"run_id:={run_id}",
                f"output_dir:={output_dir}"
            ]

            if args.dry_run:
                print(" ".join(cmd))
                continue

            # Start the process
            proc = subprocess.Popen(cmd)

            # Wait for duration.
            # The logger will exit via sys.exit(0) when it receives /experiment_done.
            # The launch file may still run if on_exit is not configured for the node,
            # so we still need a loop to poll if the process finished early or timeout happens.
            try:
                proc.wait(timeout=run_duration_sec)
            except subprocess.TimeoutExpired:
                print(f"Time limit reached ({run_duration_sec}s). Terminating run.")

            # Clean up processes to ensure nodes are fully dead before next run
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

            # Cleanup ROS 2 nodes to prevent port conflicts / orphaned nodes
            subprocess.run(["pkill", "-f", "network_proxy_node"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "experiment_logger_node"], stderr=subprocess.DEVNULL)

            time.sleep(2) # Brief pause between runs

    print("Orchestration complete! Use report_generator.py to generate tables.")

if __name__ == '__main__':
    main()
