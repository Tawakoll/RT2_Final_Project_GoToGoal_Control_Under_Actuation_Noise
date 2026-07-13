"""ROS2 experiment node for the RT2 final assignment.

The node exposes a very small JSON-over-topics interface so that a Jupyter
notebook (or any other ROS2 client) can request experimental trials and
receive the resulting metrics:

Subscribed topics
-----------------
/experiment/run  (std_msgs/String)
    JSON request, e.g.
    {"controller": "proportional", "noise_std": 0.1, "seed": 3,
     "trial_id": 42, "return_traj": true}

Published topics
----------------
/experiment/result (std_msgs/String)
    JSON reply with the request echo plus the computed metrics
    (success, time_to_goal, path_length, efficiency, final_error, goal x/y
    and, if requested, the full trajectory).

Every completed trial is also appended to a CSV file (default
``results.csv`` in the current working directory, configurable via the
``csv_path`` ROS parameter), following the good practice of always logging
experimental data in a tool-independent format.
"""

import csv
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from goal_nav_experiment.controllers import make_controller
from goal_nav_experiment.simulator import run_trial

CSV_FIELDS = ['trial_id', 'controller', 'noise_std', 'seed', 'success',
              'time_to_goal', 'path_length', 'efficiency', 'final_error',
              'goal_x', 'goal_y']


class ExperimentNode(Node):
    """Runs go-to-goal trials on request and returns/logs the metrics."""

    def __init__(self):
        super().__init__('experiment_node')
        self.declare_parameter('csv_path', 'results.csv')
        self.csv_path = self.get_parameter('csv_path').value

        self.sub = self.create_subscription(
            String, '/experiment/run', self.run_callback, 10)
        self.pub = self.create_publisher(String, '/experiment/result', 10)
        self.get_logger().info(
            f'Experiment node ready, logging to {self.csv_path}')

    def run_callback(self, msg):
        try:
            req = json.loads(msg.data)
            controller = make_controller(req.get('controller', 'proportional'))
            noise_std = float(req.get('noise_std', 0.0))
            seed = int(req.get('seed', 0))
            result = run_trial(controller, noise_std, seed)
        except Exception as exc:  # malformed request: report, don't crash
            self.get_logger().error(f'Bad request: {exc}')
            self.pub.publish(String(data=json.dumps({'error': str(exc)})))
            return

        reply = {
            'trial_id': req.get('trial_id', -1),
            'controller': controller.name,
            'noise_std': noise_std,
            'seed': seed,
            'success': bool(result['success']),
            'time_to_goal': result['time_to_goal'],
            'path_length': result['path_length'],
            'efficiency': result['efficiency'],
            'final_error': result['final_error'],
            'goal_x': result['goal_x'],
            'goal_y': result['goal_y'],
        }
        self.append_csv(reply)
        if req.get('return_traj', False):
            reply['traj_x'] = result['traj_x']
            reply['traj_y'] = result['traj_y']
        self.pub.publish(String(data=json.dumps(reply)))
        self.get_logger().info(
            f"trial {reply['trial_id']}: {controller.name} "
            f"noise={noise_std} seed={seed} -> "
            f"success={reply['success']} t={reply['time_to_goal']:.2f}"
            if reply['success'] else
            f"trial {reply['trial_id']}: {controller.name} "
            f"noise={noise_std} seed={seed} -> FAILURE")

    def append_csv(self, row):
        """Append one trial to the CSV log, writing the header if new."""
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow({k: row[k] for k in CSV_FIELDS})


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
