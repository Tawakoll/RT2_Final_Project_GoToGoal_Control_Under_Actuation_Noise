# RT2 Final Project — Closed-Loop vs. Decomposed Go-to-Goal Control Under Actuation Noise

**Research Track II — Final Assignment**
Mohamed Tawakol — MSc Robotics Engineering, University of Genoa

This repository contains the full research workflow of the RT2 final assignment:
a **research question**, a **ROS 2 experiment node** that simulates the
experiment and produces synthetic data, a **Jupyter notebook** that runs the
experimental campaign and performs the statistical analysis, and the
**IEEE-style research paper**.

## Research question

> *Does a proportional closed-loop go-to-goal controller improve time-to-goal
> and path efficiency, compared to a rotate-then-translate baseline, for a
> unicycle mobile robot navigating to random goals under increasing actuation
> noise?*

Two controllers are compared in a 2×3 full factorial design (2 controllers ×
3 noise levels), with N = 25 **paired** trials per condition (shared random
seeds → same goals and noise schedule for both controllers):

| | Proportional (proposed) | Rotate-then-translate (baseline) |
|---|---|---|
| Strategy | v and ω updated simultaneously: `v = kρ·ρ·max(cos α, 0)`, `ω = kα·α` | rotate in place while `\|α\| > 0.2 rad`, then drive straight at constant speed |
| Behaviour | smooth arc, never stops | straight lines with stop-and-rotate phases |

**Metrics:** time-to-goal, path length, path efficiency, success rate.
**Statistics:** paired t-test + Wilcoxon signed-rank + Cohen's d (α = 0.05).

## Repository structure

```
RT2_Final_Project/
├── ros2_ws/src/goal_nav_experiment/     ROS 2 (Jazzy) Python package
│   ├── goal_nav_experiment/
│   │   ├── simulator.py                 unicycle model + Gaussian actuation noise
│   │   ├── controllers.py               the two go-to-goal controllers
│   │   └── experiment_node.py           ROS 2 node (JSON over /experiment/run|result)
│   └── test/smoke_test.sh               end-to-end smoke test
├── notebook/
│   ├── RT2_experiment.ipynb             executed notebook (campaign + analysis + plots)
│   └── build_notebook.py                script that generates the notebook
├── data/
│   ├── results.csv                      raw data of all 150 trials
│   ├── summary.csv                      mean ± std per condition
│   ├── stats_*.csv                      statistical test results
│   └── figures/                         all figures used in the paper
└── paper/
    ├── paper.tex                        IEEE-style research paper (LaTeX source)
    └── paper.pdf                        compiled paper (5 pages)
```

## ROS 2 interface

The experiment node exposes a minimal JSON-over-topics interface:

- **`/experiment/run`** (`std_msgs/String`, subscribed) — trial request:
  ```json
  {"controller": "proportional", "noise_std": 0.1, "seed": 3,
   "trial_id": 42, "return_traj": true}
  ```
- **`/experiment/result`** (`std_msgs/String`, published) — metrics of the
  completed trial (success, time_to_goal, path_length, efficiency,
  final_error, goal position, and optionally the full trajectory).

Every trial is also appended to a CSV file (`csv_path` ROS parameter), so the
data can be analyzed with any external tool.

## How to reproduce

All steps run inside the course Docker image (`carms84/robeng`, ROS 2 Jazzy).

1. **Build the package**
   ```bash
   mkdir -p ~/ros2_ws/src && cp -r ros2_ws/src/goal_nav_experiment ~/ros2_ws/src/
   cd ~/ros2_ws && colcon build --symlink-install
   source install/setup.bash
   ```

2. **Install the notebook dependencies** (as per the course slides)
   ```bash
   pip3 install --break-system-packages jupyter ipywidgets pandas scipy "matplotlib==3.8.4"
   ```

3. **Run the notebook**
   ```bash
   cd notebook
   jupyter notebook --allow-root     # open RT2_experiment.ipynb and Run All
   ```
   The notebook starts the ROS 2 node itself, offers an interactive
   single-trial widget, runs the full 150-trial campaign (a few seconds),
   regenerates every figure and statistical table, and stops the node.
   Because all trials are seeded, the results are bit-for-bit reproducible.

4. **(Optional) run the smoke test**
   ```bash
   bash ros2_ws/src/goal_nav_experiment/test/smoke_test.sh
   ```

5. **(Optional) recompile the paper**
   ```bash
   cd paper && tectonic paper.tex     # or upload paper.tex + data/figures to Overleaf
   ```

## Main findings

| Hypothesis | Outcome |
|---|---|
| **H1** — proportional is faster at every noise level | **Confirmed** (≈32 % faster, paired t-test p < 0.001, \|d\| > 3) |
| **H2** — baseline is more path-efficient but degrades faster with noise | **Confirmed** (0.99 vs 0.93 noise-free; degradation −0.031 vs −0.017) |
| **H3** — the time advantage grows with noise | **Not confirmed** (p = 0.54) — an honest negative result, discussed in the paper |

## License

MIT (code). The paper and figures are © the author.
