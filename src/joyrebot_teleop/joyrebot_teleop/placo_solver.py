"""PlaCo (QP) inverse-kinematics backend, drop-in compatible with SerialChain.

XRoboToolkit's teleop sample solves IK with PlaCo, a Pinocchio-based
quadratic-programming whole-body kinematics solver. This module wraps PlaCo
behind the same interface the teleop controller already uses for SerialChain
(``names``/``lower``/``upper``/``forward``/``inverse``), so the two solvers can
be swapped and compared through the ``ik_solver`` parameter.

Why QP instead of the built-in damped-least-squares (DLS) solver:
  * Joint (and velocity) limits enter as hard QP *constraints* instead of a
    post-hoc ``np.clip``, so when a joint saturates the solver redistributes the
    requested motion to the remaining joints instead of failing and freezing.
  * A manipulability task steers the arm away from singular / limit corners.
  * The solve is best-effort and always returns a feasible motion, so the end
    effector keeps tracking (e.g. wrist pitch) up to the joint limit rather than
    stalling the moment the DLS iteration can no longer converge.

PlaCo is an optional dependency (``pip install placo``). It is imported lazily,
so the default DLS path keeps working without it.
"""

import numpy as np

from .kinematics import SerialChain


class PlacoChain:
    """QP IK backend exposing the subset of the SerialChain API the node uses."""

    def __init__(self, urdf_path, base_link="base_link", tip_link="gripper_end",
                 dt=1.0 / 60.0, task_weight=1.0, manipulability_weight=0.0,
                 enable_limits=True, solve_iterations=50):
        try:
            import placo
        except ImportError as error:  # pragma: no cover - optional dependency
            raise ImportError(
                "ik_solver='placo' requires the 'placo' package (pip install placo)"
            ) from error

        # Reuse the URDF parser for joint names and limits so the reported soft
        # limits (and the controller's home validation) match the DLS path 1:1.
        reference = SerialChain.from_urdf(urdf_path, base_link=base_link, tip_link=tip_link)
        self.names = list(reference.names)
        self.lower = reference.lower.copy()
        self.upper = reference.upper.copy()

        self.tip_link = tip_link
        self.solve_iterations = int(solve_iterations)

        self.robot = placo.RobotWrapper(str(urdf_path))
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = float(dt)

        # The URDF's base_link is not welded to the world, so placo loads the arm
        # with a free-floating 6-DoF base (state.q has 7 extra values). Left free,
        # the IK cheats a Cartesian target by *translating the whole robot* while
        # the arm joints barely move -- on a bolted-down arm that means the end
        # effector does not follow at all (e.g. pushing the stick forward does
        # nothing). Lock the floating base so only the joints solve the task.
        if self.robot.state.q.shape[0] > len(self.names) and hasattr(self.solver, "mask_fbase"):
            self.solver.mask_fbase(True)

        if enable_limits:
            # Respect the URDF joint and velocity limits as real QP constraints.
            self.solver.enable_joint_limits(True)
            self.solver.enable_velocity_limits(True)

        # Primary 6-DoF end-effector pose task (soft, high weight).
        self.frame_task = self.solver.add_frame_task(tip_link, self.robot.get_T_world_frame(tip_link))
        self.frame_task.configure("ee_pose", "soft", float(task_weight))

        # Optional manipulability task. DISABLED by default: on a non-redundant
        # 6-DoF arm the 6-DoF frame task already fixes every joint, so there is no
        # null space for a secondary objective to exploit. Adding it as a competing
        # soft task instead trades pose accuracy for manipulability every tick, which
        # makes the arm drift continuously (never settling, "moving on startup" and
        # sliding away right after a home reset). Only enable it for redundant /
        # whole-body robots (7+ DoF), which is what XRoboToolkit's samples target.
        if manipulability_weight > 0.0:
            manipulability = self.solver.add_manipulability_task(tip_link, "both", 1.0)
            manipulability.configure("manipulability", "soft", float(manipulability_weight))

    def _set_q(self, q):
        for name, value in zip(self.names, np.asarray(q, dtype=float)):
            self.robot.set_joint(name, float(value))
        self.robot.update_kinematics()

    def _get_q(self):
        return np.asarray([self.robot.get_joint(name) for name in self.names], dtype=float)

    def _tip_pose(self):
        return np.asarray(self.robot.get_T_world_frame(self.tip_link), dtype=float)

    def forward(self, q):
        self._set_q(q)
        return self._tip_pose()

    def inverse(self, target, seed, position_tolerance=0.004,
                orientation_tolerance=0.015, **_):
        """Best-effort QP IK. Always returns a feasible pose (it never freezes).

        ``damping`` / ``max_iterations`` from the DLS call signature are accepted
        and ignored so this stays a drop-in replacement.
        """
        target = np.asarray(target, dtype=float)
        self._set_q(seed)
        self.frame_task.T_world_frame = target
        for _ in range(self.solve_iterations):
            self.solver.solve(True)
            self.robot.update_kinematics()
            error = SerialChain.pose_error(self._tip_pose(), target)
            if (np.linalg.norm(error[:3]) <= position_tolerance
                    and np.linalg.norm(error[3:]) <= orientation_tolerance):
                break
        q = self._get_q()
        # QP is best-effort: report success whenever the result is finite so the
        # controller keeps moving toward the limit instead of holding the pose.
        return q, bool(np.all(np.isfinite(q)))
