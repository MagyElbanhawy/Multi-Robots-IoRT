# Reviewer Responses and Revisions

This document contains text and tables drafted in response to the reviewers' comments, suitable for direct integration into the journal revision and response letter.

---

## 1. Computational Complexity Analysis

**Response to Reviewer 1:**
*“Complexity analysis and potential hardware bottlenecks should be included.”*

### Draft Section for Paper:

**Computational Complexity and Hardware Bottlenecks**

To evaluate the feasibility of deploying the EMRMF framework on resource-constrained multi-robot systems, we analyze the computational complexity of its core modules. Let $N$ be the number of poses in the local trajectory graph, $M$ be the number of inter-robot observation constraints, $K$ be the number of features/submap points being fused, and $E$ be the total number of edges in the global optimization graph.

| Module | Asymptotic Complexity | Description |
| :--- | :--- | :--- |
| **Local Graph SLAM** | $\mathcal{O}(N \log N)$ | Standard pose-graph optimization on individual robots. |
| **Trust Factor Computation** | $\mathcal{O}(M)$ | Calculating $\theta$ using prediction-observation deviation. |
| **Map Fusion** | $\mathcal{O}(K^2)$ | Nearest-neighbor matching for point cloud/submap alignment. |
| **Global Optimization** | $\mathcal{O}(E)$ | Sparse Cholesky factorization of the global pose graph. |

As shown in the table above, the **Trust Factor Computation** is highly lightweight ($\mathcal{O}(M)$) because evaluating the trust metric requires only a deterministic mathematical calculation (the cubic error formulation and exponential time decay) for each incoming constraint. It adds negligible overhead to the system.

The primary computational bottlenecks remain the **Map Fusion** ($\mathcal{O}(K^2)$ due to feature matching) and **Global Optimization**. However, the EMRMF framework mitigates hardware bottlenecks on individual robots through an edge-server architecture. Heavy computational tasks—specifically map fusion and large-scale global optimization—are offloaded to the central server. The robots themselves only compute local odometry, local graph SLAM, and the lightweight trust factor parameters, easily running in real-time on edge processors (e.g., Raspberry Pi or Jetson platforms) without stalling.

---

## 2. Multi-Robot SLAM Comparison

**Response to Reviewer 3:**
*“More directly related multi-robot SLAM baselines should be included.”*

### Draft Table for Paper:

We have added a qualitative comparison table summarizing how EMRMF positions itself against state-of-the-art multi-robot SLAM baselines.

**Table: Qualitative Comparison of Multi-Robot SLAM Frameworks**

| Framework | Architecture | Communication Robustness | Trust / Outlier Rejection | Map Representation |
| :--- | :--- | :--- | :--- | :--- |
| **CSM (Conventional)** | Centralized | Low (Assumes perfect comms) | Standard robust kernels (Huber/Cauchy) | Occupancy Grid / Points |
| **DOOR-SLAM** | Decentralized | Medium (Peer-to-peer data limits) | Pairwise Consistency Maximization (PCM) | Pose Graph & Features |
| **Kimera-Multi** | Decentralized | High (Bandwidth-aware routing) | Graduated Non-Convexity (GNC) | 3D Mesh / Metric-Semantic |
| **Multi-Robot RTAB-Map** | Client-Server | Low (Drops frames on lag) | Standard graph optimization | 3D Point Cloud / Octree |
| **EMRMF (Proposed)** | Client-Server | **High (Delay/Loss tolerant)** | **Dynamic Trust Factor ($\theta$)** | 3D Point Cloud & Features |

**Discussion:**
While decentralized frameworks like *DOOR-SLAM* and *Kimera-Multi* introduce robust outlier rejection mechanisms (PCM and GNC, respectively) to handle perceptual aliasing and false loop closures, they lack an explicit temporal and spatial dynamic trust metric to handle underlying *network degradation* (e.g., severe packet loss or latency from LoRa/Wi-Fi). *Multi-Robot RTAB-Map* provides excellent client-server map fusion but assumes relatively stable networks, often failing to integrate heavily delayed submaps smoothly. The proposed **EMRMF** bridges this gap. By utilizing the explicit trust factor $\theta$, EMRMF actively modulates the weight of inter-robot constraints based on communication health (delay) and kinematic consistency (error), making it distinctly suited for highly degraded IoT communication environments.
