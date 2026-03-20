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