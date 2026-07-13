"""Replay goal_nav_experiment trials inside Gazebo Sim.

The experiment itself always runs in the kinematic simulator
(``simulator.run_trial``), so the metrics are byte-identical to the ones
reported in the notebook and the paper. This script only adds a visual
layer: after computing a trial it re-plays the recorded trajectory in a
Gazebo world at real-time speed by teleporting the ``robot`` model through
the ``/world/arena/set_pose`` service (UserCommands system), and moves the
``goal`` marker to the trial's goal position.

Usage (inside the container, with the arena world already running):

    # terminal 1
    gz sim -r $(ros2 pkg prefix goal_nav_experiment)/share/goal_nav_experiment/worlds/arena.sdf

    # terminal 2
    ros2 run goal_nav_experiment gazebo_viz --controller proportional --noise 0.1 --seed 3
    ros2 run goal_nav_experiment gazebo_viz --controller rotate_translate --noise 0.1 --seed 3

Options:
    --speed 2.0     replay at 2x real time (useful for slow baseline trials)
    --loop          replay the trial forever until Ctrl+C
"""

import argparse
import math
import sys
import time

from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.pose_pb2 import Pose
from gz.transport14 import Node

from .controllers import CONTROLLERS, make_controller
from .simulator import DT, START_THETA, START_X, START_Y, run_trial

WORLD = 'arena'
SET_POSE_SERVICE = f'/world/{WORLD}/set_pose'
SERVICE_TIMEOUT_MS = 2000


def set_entity_pose(node, name, x, y, z, yaw=0.0):
    """Teleport a model through the world's set_pose service."""
    req = Pose()
    req.name = name
    req.position.x = float(x)
    req.position.y = float(y)
    req.position.z = float(z)
    req.orientation.z = math.sin(yaw / 2.0)
    req.orientation.w = math.cos(yaw / 2.0)
    ok, reply = node.request(SET_POSE_SERVICE, req, Pose, Boolean,
                             SERVICE_TIMEOUT_MS)
    return ok and reply.data


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Replay a go-to-goal trial in Gazebo Sim.')
    parser.add_argument('--controller', default='proportional',
                        choices=sorted(CONTROLLERS),
                        help='controller to run (default: proportional)')
    parser.add_argument('--noise', type=float, default=0.0,
                        help='actuation noise std as fraction of the '
                             'velocity limits (default: 0.0)')
    parser.add_argument('--seed', type=int, default=0,
                        help='RNG seed: same seed => same goal and noise '
                             'schedule for both controllers (default: 0)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='replay speed factor, 1.0 = real time '
                             '(default: 1.0)')
    parser.add_argument('--loop', action='store_true',
                        help='replay the trial in a loop until Ctrl+C')
    args = parser.parse_args(argv)

    # 1. run the trial in the kinematic simulator (ground truth)
    result = run_trial(make_controller(args.controller), args.noise,
                       args.seed)
    n_steps = len(result['traj_x'])
    print(f"controller: {args.controller}   noise: {args.noise}   "
          f"seed: {args.seed}")
    print(f"goal: ({result['goal_x']:.2f}, {result['goal_y']:.2f})   "
          f"success: {result['success']}   "
          f"time to goal: {result['time_to_goal']:.2f} s   "
          f"path length: {result['path_length']:.2f} m   "
          f"efficiency: {result['efficiency']:.3f}")

    # 2. connect to the running Gazebo world
    node = Node()
    if not set_entity_pose(node, 'goal',
                           result['goal_x'], result['goal_y'], 0.03):
        sys.exit(f'could not reach {SET_POSE_SERVICE} -- is the arena '
                 'world running? (gz sim -r .../worlds/arena.sdf)')

    # 3. replay the trajectory at real-time speed (scaled by --speed)
    step_period = DT / args.speed
    try:
        while True:
            set_entity_pose(node, 'robot', START_X, START_Y, 0.09,
                            START_THETA)
            time.sleep(0.8)
            t_next = time.monotonic()
            for i in range(n_steps):
                set_entity_pose(node, 'robot',
                                result['traj_x'][i], result['traj_y'][i],
                                0.09, result['traj_theta'][i])
                t_next += step_period
                delay = t_next - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            print(f'replay finished ({n_steps} steps, '
                  f'{n_steps * DT:.1f} s simulated).')
            if not args.loop:
                break
            time.sleep(1.5)
    except KeyboardInterrupt:
        print('\nreplay interrupted.')


if __name__ == '__main__':
    main()
