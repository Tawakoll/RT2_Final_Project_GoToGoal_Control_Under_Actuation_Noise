"""Go-to-goal controllers compared in the experiment.

Both controllers receive the polar coordinates of the goal in the robot
frame -- the distance ``rho`` and the bearing error ``alpha`` -- and return
the commanded linear and angular velocities (v, w).

* ``ProportionalController`` (proposed): the classical closed-loop polar
  law (Aicardi et al., 1995): v and w are updated simultaneously and
  continuously, so heading errors are corrected while the robot advances.

* ``RotateThenTranslateController`` (baseline): the naive decomposed
  strategy: rotate in place until the heading error is below a threshold,
  then translate at constant speed; re-rotate whenever the error grows
  beyond the threshold.
"""


class ProportionalController:
    """Simultaneous proportional control of v and w (proposed approach)."""

    name = 'proportional'

    def __init__(self, k_rho=1.2, k_alpha=3.0):
        self.k_rho = k_rho
        self.k_alpha = k_alpha

    def compute(self, rho, alpha):
        # scale the forward speed down when badly oriented (cos gating,
        # clipped at zero so the robot never drives backwards)
        import math
        v = self.k_rho * rho * max(math.cos(alpha), 0.0)
        w = self.k_alpha * alpha
        return v, w


class RotateThenTranslateController:
    """Rotate in place, then translate (baseline approach)."""

    name = 'rotate_translate'

    def __init__(self, alpha_threshold=0.2, v_const=1.0, w_const=1.5):
        self.alpha_threshold = alpha_threshold
        self.v_const = v_const
        self.w_const = w_const

    def compute(self, rho, alpha):
        if abs(alpha) > self.alpha_threshold:
            w = self.w_const if alpha > 0 else -self.w_const
            return 0.0, w
        return self.v_const, 0.0


CONTROLLERS = {
    ProportionalController.name: ProportionalController,
    RotateThenTranslateController.name: RotateThenTranslateController,
}


def make_controller(name):
    """Instantiate a controller by name ('proportional' or 'rotate_translate')."""
    try:
        return CONTROLLERS[name]()
    except KeyError:
        raise ValueError(
            f'Unknown controller "{name}". Valid: {list(CONTROLLERS)}')
