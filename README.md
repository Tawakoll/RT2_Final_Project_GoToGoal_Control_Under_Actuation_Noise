# RT2 Final Project — Closed-Loop vs. Decomposed Go-to-Goal Control Under Actuation Noise

**Research Track II — Final Assignment**
Mohamed Tawakol — MSc Robotics Engineering, University of Genoa

Work in progress.

This repository contains the research workflow of the RT2 final assignment:
a ROS 2 experiment node that simulates go-to-goal navigation of a unicycle
robot under actuation noise, comparing a proportional closed-loop controller
against a rotate-then-translate baseline, plus a Gazebo arena for visualizing
trials, a Jupyter notebook that runs the experimental campaign, and the
IEEE-style research paper (draft).

Structure:
- `ros2_ws/` — ROS 2 package `goal_nav_experiment` (simulator, controllers, experiment node, Gazebo visualization)
- `notebook/` — Jupyter notebook driving the experiments and analysis
- `data/` — experiment results (CSV) and figures
- `paper/` — IEEE-style paper (draft in progress)
