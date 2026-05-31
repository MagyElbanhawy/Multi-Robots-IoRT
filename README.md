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

To launch the system, you can use the provided shell scripts for each phase. Ensure you give execution permissions to these scripts first.

```bash
chmod +x phase1_local_mapping.sh
chmod +x phase2_trust_factor.sh
chmod +x phase3_global_fusion.sh
```

### Phase 1: Decentralized Local Mapping

This phase initializes the hardware sensors and sets up the local coordinate frames using RTAB-Map.

To launch:
```bash
./phase1_local_mapping.sh
```

### Phase 2: Map Collecting Using the Trust Factor ($\theta$)

This phase introduces the trust factor to mitigate the effects of communication delays and sensor noise on pose estimation.

*Note: The actual implementation of the dynamic trust factor $\theta$ will require a custom ROS 2 node or an extension to the existing RTAB-Map pose graph optimizer. The script currently starts odometry nodes and contains placeholders for running the trust factor node and LoRa communication.*

To launch:
```bash
./phase2_trust_factor.sh
```

### Phase 3: Multi-Robot Global Map Fusion

This phase handles combining individual local maps from each robot into a unified global map. It is designed to run on a central server.

*Note: This script contains placeholders for the custom EMRMF global map fusion and dynamic task allocation nodes.*

To launch:
```bash
./phase3_global_fusion.sh
```

## EMRMF Experiment Logger for Journal Revisions

The `emrmf_core` package includes a ROS 2 Humble console entry point for running
repeatable EMRMF multi-robot SLAM experiment sweeps:

```bash
ros2 run emrmf_core experiment_logger --output-dir emrmf_experiment_logs
```

The logger evaluates a full factorial grid over:

- Trust-factor spatial exponents `p` and temporal decay rates `gamma`.
- Artificial communication delays of `0.5 s` and `2.0 s` by default.
- Artificial packet-loss rates of `10%` and `30%` by default.
- Five repeated runs per configuration by default.
- Ablation modes:
  - `baseline_graph_slam`
  - `decentralized_only`
  - `trust_only`
  - `full_emrmf`

For every repeated run, the logger computes the EMRMF trust factor

```text
theta = max(0, 1 - (norm(e_ij) / tau_e)^p) * exp(-gamma * delta_t)
```

and records pose RMSE, map-alignment RMSE, fusion time, injected delay,
injected packet loss, `theta_mean`, and `theta_std`.  It creates a timestamped
session folder under the output directory and writes publication-ready CSV files:

- `run_logs/<timestamp>_<configuration>_runNN.csv`: one timestamped CSV log for
  every individual run.
- `emrmf_experiment_results_<session>.csv`: all repeated-run rows with links to
  their per-run CSV logs.
- `emrmf_experiment_summary_<session>.csv`: the final Springer-revision summary
  CSV containing mean, standard deviation, and 95% confidence interval for each
  metric grouped by configuration.

Example journal-revision sweep with explicit parameters:

```bash
ros2 run emrmf_core experiment_logger \
  --output-dir results/emrmf_revision \
  --p-values 1.0 2.0 3.0 \
  --gamma-values 0.0 0.25 0.5 1.0 \
  --delays 0.5 2.0 \
  --packet-losses 0.10 0.30 \
  --repeats 5
```

Use `--session-id springer_revision_trial_01` when you need stable, named
folders for a revision archive; otherwise the logger uses the current UTC
timestamp as the session identifier. The generated folder is the file bundle to
attach to the experimental-validation section of the revision.



### One-command Springer revision run

Use the included runner when you want the repository to create the complete file
bundle without remembering the long `ros2 run` command:

```bash
./run_emrmf_experiments.sh
```

By default it saves to `results/emrmf_revision/<UTC session>/`, with one
`timestamped` CSV per run in `run_logs/`, an aggregate repeated-run CSV, and the
final summary CSV. You can name a revision archive explicitly:

```bash
OUTPUT_DIR=results/emrmf_revision SESSION_ID=springer_revision_trial_01 ./run_emrmf_experiments.sh
```

The current metric source is deterministic and self-contained so the full CSV
pipeline can be exercised before hardware or rosbag playback is available.  To
use live robots, replace the `MetricSimulator` source in
`emrmf_core.experiment_logger` with measurements from ROS topics or bags while
keeping the same result and summary CSV schema.
