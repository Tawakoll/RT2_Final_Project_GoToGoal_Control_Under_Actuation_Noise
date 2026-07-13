"""Unicycle kinematic simulator with actuation noise.

The simulator integrates the standard unicycle model

    x_{k+1}     = x_k + v * cos(theta_k) * dt
    y_{k+1}     = y_k + v * sin(theta_k) * dt
    theta_{k+1} = theta_k + w * dt

where the executed velocities (v, w) differ from the commanded ones by
additive zero-mean Gaussian noise, emulating imperfect actuation:

    v = v_cmd + N(0, sigma_v)      sigma_v = noise_std * V_MAX
    w = w_cmd + N(0, sigma_w)      sigma_w = noise_std * W_MAX

All randomness is driven by a seeded numpy Generator so that every trial is
exactly reproducible, and the same seed can be reused across controllers to
obtain paired samples (same goal, same noise realization schedule).
"""

import math

import numpy as np

# Arena and robot limits (turtlesim-like 11 x 11 arena)
ARENA_MIN = 0.0
ARENA_MAX = 11.0
START_X = 5.5
START_Y = 5.5
START_THETA = 0.0
V_MAX = 2.0      # m/s
W_MAX = 2.0      # rad/s
DT = 0.05        # integration step [s]
T_MAX = 60.0     # trial timeout [s]
GOAL_TOL = 0.15  # goal reached when distance < GOAL_TOL [m]
MIN_GOAL_DIST = 3.0  # minimum start-goal distance when sampling goals


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def sample_goal(rng):
    """Sample a goal uniformly in [1, 10]^2, at least MIN_GOAL_DIST from start."""
    while True:
        gx = rng.uniform(ARENA_MIN + 1.0, ARENA_MAX - 1.0)
        gy = rng.uniform(ARENA_MIN + 1.0, ARENA_MAX - 1.0)
        if math.hypot(gx - START_X, gy - START_Y) >= MIN_GOAL_DIST:
            return gx, gy


def run_trial(controller, noise_std, seed, goal=None):
    """Run a single go-to-goal trial and return metrics and trajectory.

    Parameters
    ----------
    controller : object with a ``compute(rho, alpha)`` method returning (v, w)
    noise_std  : actuation noise level, as a fraction of the velocity limits
    seed       : RNG seed (controls goal position and noise realization)
    goal       : optional (gx, gy); if None it is sampled from the seeded RNG

    Returns
    -------
    dict with keys: success, time_to_goal, path_length, efficiency,
    final_error, goal_x, goal_y, traj_x, traj_y, traj_theta
    """
    rng = np.random.default_rng(seed)
    if goal is None:
        goal = sample_goal(rng)
    gx, gy = goal

    x, y, theta = START_X, START_Y, START_THETA
    straight_dist = math.hypot(gx - x, gy - y)
    path_length = 0.0
    t = 0.0
    traj_x, traj_y, traj_theta = [x], [y], [theta]
    success = False

    while t < T_MAX:
        rho = math.hypot(gx - x, gy - y)
        if rho < GOAL_TOL:
            success = True
            break
        alpha = wrap_angle(math.atan2(gy - y, gx - x) - theta)

        v_cmd, w_cmd = controller.compute(rho, alpha)
        v_cmd = min(max(v_cmd, 0.0), V_MAX)
        w_cmd = min(max(w_cmd, -W_MAX), W_MAX)

        # actuation noise (drawn every step, also when velocity is zero,
        # so that paired trials share the same noise schedule)
        dv = rng.normal(0.0, noise_std * V_MAX)
        dw = rng.normal(0.0, noise_std * W_MAX)
        v = max(v_cmd + dv, 0.0)
        w = w_cmd + dw

        nx = x + v * math.cos(theta) * DT
        ny = y + v * math.sin(theta) * DT
        nx = min(max(nx, ARENA_MIN), ARENA_MAX)
        ny = min(max(ny, ARENA_MIN), ARENA_MAX)
        path_length += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        theta = wrap_angle(theta + w * DT)
        t += DT
        traj_x.append(x)
        traj_y.append(y)
        traj_theta.append(theta)

    final_error = math.hypot(gx - x, gy - y)
    # path efficiency: distance actually covered towards the goal divided by
    # the distance travelled (1 = straight line, <1 = detours/oscillations)
    covered = max(straight_dist - final_error, 0.0)
    return {
        'success': success,
        'time_to_goal': t if success else float('nan'),
        'path_length': path_length,
        'efficiency': covered / path_length if path_length > 0 else 0.0,
        'final_error': final_error,
        'goal_x': gx,
        'goal_y': gy,
        'traj_x': traj_x,
        'traj_y': traj_y,
        'traj_theta': traj_theta,
    }
