"""Constrained incremental inverse kinematics backed by PlaCo.

The solver is deliberately used as a velocity-level controller: one call to
``inverse`` advances the model by exactly one controller period.  Optional
non-linear re-linearisation substeps divide that period instead of multiplying
it.  This keeps PlaCo's joint-velocity constraints consistent with the command
rate.

PlaCo is imported lazily so the built-in DLS backend remains usable without the
optional dependency.
"""

from dataclasses import dataclass

import numpy as np

from .kinematics import SerialChain


@dataclass(frozen=True)
class IKDiagnostics:
    """Diagnostics from the most recent QP update."""

    position_error: float = float("inf")
    orientation_error: float = float("inf")
    target_reached: bool = False
    at_joint_limit: bool = False
    velocity_limited: bool = False
    iterations: int = 0
    failure: str = ""


class PlacoChain:
    """QP IK backend exposing the subset of ``SerialChain`` used by the node."""

    def __init__(
            self, urdf_path, base_link="base_link", tip_link="gripper_end",
            dt=1.0 / 60.0, position_weight=100.0, orientation_weight=0.35,
            manipulability_weight=0.0, joint_margin=0.02,
            max_joint_speed=0.7, enable_limits=True, solve_iterations=1):
        try:
            import placo
        except ImportError as error:  # pragma: no cover - optional dependency
            raise ImportError(
                "ik_solver='placo' requires the 'placo' package "
                "(install joyrebot_teleop[placo])"
            ) from error

        self.dt = float(dt)
        self.max_joint_speed = float(max_joint_speed)
        self.solve_iterations = int(solve_iterations)
        position_weight = float(position_weight)
        orientation_weight = float(orientation_weight)
        manipulability_weight = float(manipulability_weight)
        joint_margin = float(joint_margin)
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if not np.isfinite(self.max_joint_speed) or self.max_joint_speed <= 0.0:
            raise ValueError("max_joint_speed must be finite and positive")
        if self.solve_iterations < 1:
            raise ValueError("solve_iterations must be at least one")
        if joint_margin < 0.0 or not np.isfinite(joint_margin):
            raise ValueError("joint_margin must be finite and non-negative")
        if (position_weight < 0.0 or orientation_weight < 0.0
                or not np.isfinite(position_weight)
                or not np.isfinite(orientation_weight)
                or position_weight + orientation_weight <= 0.0):
            raise ValueError("IK task weights must be finite, non-negative, and not both zero")
        if manipulability_weight < 0.0 or not np.isfinite(manipulability_weight):
            raise ValueError("manipulability_weight must be finite and non-negative")

        # Share the same serial-chain parsing and public limits as the DLS path.
        reference = SerialChain.from_urdf(
            urdf_path, base_link=base_link, tip_link=tip_link)
        self.names = list(reference.names)
        self.lower = reference.lower.copy() + joint_margin
        self.upper = reference.upper.copy() - joint_margin
        if np.any(self.lower >= self.upper):
            raise ValueError("joint_margin leaves one or more joints without a valid range")

        self.tip_link = tip_link
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.last_diagnostics = IKDiagnostics()

        self.robot = placo.RobotWrapper(str(urdf_path))

        # PlaCo reads limits from RobotWrapper. Override the very high URDF
        # velocity values with the actual teleoperation limit and move the hard
        # position constraints inward by the configured safety margin.
        if enable_limits:
            for name, lower, upper in zip(self.names, self.lower, self.upper):
                self.robot.set_joint_limits(name, float(lower), float(upper))
                self.robot.set_velocity_limit(name, self.max_joint_speed)

        self.solver = placo.KinematicsSolver(self.robot)
        # N re-linearisation steps still represent one controller period.
        self.solver.dt = self.dt / self.solve_iterations

        state_q = np.asarray(self.robot.state.q)
        has_floating_base = state_q.size > len(self.names)
        if has_floating_base:
            # Some PlaCo/model combinations initialise the free-base quaternion
            # to zero. Ensure a valid identity pose before fixing the base.
            if np.linalg.norm(state_q[3:7]) < 1e-12:
                self.robot.state.q[:7] = np.array(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            if hasattr(self.solver, "mask_fbase"):
                self.solver.mask_fbase(True)

        if enable_limits:
            self.solver.enable_joint_limits(True)
            self.solver.enable_velocity_limits(True)

        self.robot.update_kinematics()
        self.frame_task = self.solver.add_frame_task(
            tip_link, self.robot.get_T_world_frame(tip_link))
        # Position and orientation have different units and different practical
        # importance in teleoperation, so configure their weights independently.
        self.frame_task.configure(
            "ee_pose", "soft", position_weight, orientation_weight)

        # A manipulability objective competes with the pose task on this
        # non-redundant 6-DoF arm. Keep it opt-in; it is mainly useful after
        # adding redundant DoFs or deliberately accepting pose error.
        self.manipulability_task = None
        if manipulability_weight > 0.0:
            self.manipulability_task = self.solver.add_manipulability_task(
                tip_link, "both", 1.0)
            self.manipulability_task.configure(
                "manipulability", "soft", manipulability_weight)

    def _set_q(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != self.lower.shape:
            raise ValueError(
                f"Expected {len(self.names)} joint values, got shape {q.shape}")
        for name, value in zip(self.names, q):
            self.robot.set_joint(name, float(value))
        self.robot.update_kinematics()

    def _get_q(self):
        return np.asarray(
            [self.robot.get_joint(name) for name in self.names], dtype=float)

    def _tip_pose(self):
        return np.asarray(
            self.robot.get_T_world_frame(self.tip_link), dtype=float)

    def forward(self, q):
        self._set_q(q)
        return self._tip_pose()

    def _failed(self, seed, failure, iterations=0):
        self._set_q(seed)
        self.last_diagnostics = IKDiagnostics(
            iterations=iterations, failure=str(failure))
        return seed.copy(), False

    def inverse(
            self, target, seed, position_tolerance=0.004,
            orientation_tolerance=0.015, **_):
        """Advance one safe QP control period toward ``target``.

        The Boolean return value means that PlaCo produced a finite step obeying
        the configured position and velocity constraints.  It does *not* mean
        the target has already been reached; convergence is exposed through
        ``last_diagnostics.target_reached`` so unreachable soft targets can
        still be approached instead of freezing the arm.
        """
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if seed.shape != self.lower.shape:
            raise ValueError(
                f"Expected {len(self.names)} seed values, got shape {seed.shape}")
        if not np.all(np.isfinite(seed)):
            # The controller will reject this result and retain its last valid
            # command. Avoid feeding NaN/Inf into the native QP backend.
            safe_seed = np.clip(
                np.nan_to_num(seed, nan=0.0, posinf=0.0, neginf=0.0),
                self.lower, self.upper)
            return self._failed(safe_seed, "non-finite seed")
        safe_seed = np.clip(seed, self.lower, self.upper)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            return self._failed(safe_seed, "invalid target transform")

        position_tolerance = float(position_tolerance)
        orientation_tolerance = float(orientation_tolerance)
        if position_tolerance <= 0.0 or orientation_tolerance <= 0.0:
            raise ValueError("IK tolerances must be positive")

        self._set_q(safe_seed)
        self.frame_task.T_world_frame = target
        completed_iterations = 0
        try:
            for completed_iterations in range(1, self.solve_iterations + 1):
                self.solver.solve(True)
                self.robot.update_kinematics()
        except RuntimeError as error:
            return self._failed(
                safe_seed, f"QP solve failed: {error}", completed_iterations)

        q = self._get_q()
        if not np.all(np.isfinite(q)):
            return self._failed(
                safe_seed, "QP returned non-finite joints", completed_iterations)

        # These checks are redundant with the QP constraints by design: they
        # form a final trust boundary before a native-solver result reaches ROS.
        limit_epsilon = 1e-7
        if (np.any(q < self.lower - limit_epsilon)
                or np.any(q > self.upper + limit_epsilon)):
            return self._failed(
                safe_seed, "QP violated a joint limit", completed_iterations)
        max_step = self.max_joint_speed * self.dt
        step = q - safe_seed
        if np.any(np.abs(step) > max_step + limit_epsilon):
            return self._failed(
                safe_seed, "QP violated the velocity limit", completed_iterations)

        # Remove harmless floating-point excursions at an active boundary.
        q = np.clip(q, self.lower, self.upper)
        self._set_q(q)
        error = SerialChain.pose_error(self._tip_pose(), target)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = float(np.linalg.norm(error[3:]))
        at_joint_limit = bool(np.any(
            np.minimum(q - self.lower, self.upper - q) <= 1e-5))
        velocity_limited = bool(np.any(
            np.abs(q - safe_seed) >= max_step * (1.0 - 1e-4)))
        target_reached = bool(
            position_error <= position_tolerance
            and orientation_error <= orientation_tolerance)
        self.last_diagnostics = IKDiagnostics(
            position_error=position_error,
            orientation_error=orientation_error,
            target_reached=target_reached,
            at_joint_limit=at_joint_limit,
            velocity_limited=velocity_limited,
            iterations=completed_iterations)
        return q, True
