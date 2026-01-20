"""
Base motion controller with S-curve trajectory planning for high-precision positioning.
"""

from abc import ABC, abstractmethod
from typing import Optional, List, Tuple, Dict, Any
import numpy as np
import time
from dataclasses import dataclass

from .data_structures import (
    Vector3D, StagePosition, AxisLimits, MotionState,
    BoundingBox
)


@dataclass
class MotionParameters:
    """Parameters for motion planning."""
    max_velocity: float  # Maximum velocity (microns/s)
    max_acceleration: float  # Maximum acceleration (microns/s²)
    max_jerk: float  # Maximum jerk (microns/s³)
    position_tolerance: float = 0.1  # Position tolerance (microns)
    velocity_tolerance: float = 1.0  # Velocity tolerance (microns/s)


@dataclass
class TrajectorySegment:
    """Segment of a planned trajectory."""
    duration: float  # Duration of this segment (seconds)
    initial_position: Vector3D
    initial_velocity: Vector3D
    initial_acceleration: Vector3D
    final_position: Vector3D
    final_velocity: Vector3D
    final_acceleration: Vector3D


class SCurvePlanner:
    """S-curve trajectory planner for smooth motion."""

    def __init__(self, motion_params: MotionParameters):
        self.motion_params = motion_params

    def plan_1d_move(self, initial_pos: float, final_pos: float,
                     initial_vel: float = 0.0, final_vel: float = 0.0) -> List[float]:
        """
        Plan a 1D move using S-curve trajectory.
        Returns a list of time points for the trajectory segments.
        """
        distance = final_pos - initial_pos

        if abs(distance) < self.motion_params.position_tolerance:
            return [0.0]  # No movement needed

        # If we're already at the target velocity, we need to adjust
        if abs(initial_vel - final_vel) < self.motion_params.velocity_tolerance:
            if initial_vel == 0:
                return [distance, self._calculate_move_time(distance)]

        # Calculate the trajectory phases
        # 1. Acceleration phase (jerk limited)
        # 2. Constant acceleration phase
        # 3. Deceleration phase (jerk limited)
        # 4. Constant velocity phase (if needed)
        # 5. Deceleration to target phase

        # Simplified S-curve with 7 phases
        return self._calculate_s_curve_phases(initial_pos, final_pos, initial_vel, final_vel)

    def _calculate_s_curve_phases(self, initial_pos: float, final_pos: float,
                                 initial_vel: float, final_vel: float) -> List[float]:
        """Calculate the 7 phases of S-curve trajectory."""
        distance = final_pos - initial_pos

        # Calculate reachable velocity
        max_vel_sq = 2 * self.motion_params.max_acceleration * abs(distance)
        reachable_velocity = min(
            self.motion_params.max_velocity,
            np.sqrt(max_vel_sq)
        )

        # Time calculations for S-curve
        # Phase 1: Jerk increase to max acceleration
        t1 = self.motion_params.max_acceleration / self.motion_params.max_jerk

        # Phase 2: Constant acceleration
        t2 = (reachable_velocity - initial_vel) / self.motion_params.max_acceleration - t1

        # Phase 3: Jerk decrease to zero (end acceleration)
        t3 = t1  # Symmetric with phase 1

        # Phase 4: Constant velocity
        accel_distance = (initial_vel + reachable_velocity) * (t1 + t2 + t3) / 2
        const_vel_distance = distance - 2 * accel_distance  # Account for deceleration
        t4 = const_vel_distance / reachable_velocity if reachable_velocity > 0 else 0

        # Phases 5-7: Deceleration (symmetric to 1-3)
        t5 = t1
        t6 = t2
        t7 = t1

        return [t1, t2, t3, t4, t5, t6, t7]

    def _calculate_move_time(self, distance: float) -> float:
        """Calculate total move time for a simple trapezoidal profile."""
        if distance == 0:
            return 0.0

        # Check if we can reach max velocity
        accel_time = self.motion_params.max_velocity / self.motion_params.max_acceleration
        accel_distance = 0.5 * self.motion_params.max_acceleration * accel_time ** 2

        if 2 * accel_distance > abs(distance):
            # Triangular profile (no constant velocity phase)
            return 2 * np.sqrt(abs(distance) / self.motion_params.max_acceleration)
        else:
            # Trapezoidal profile
            const_vel_distance = abs(distance) - 2 * accel_distance
            const_vel_time = const_vel_distance / self.motion_params.max_velocity
            return 2 * accel_time + const_vel_time

    def interpolate_position(self, trajectory: List[float], t: float,
                           initial_pos: Vector3D, final_pos: Vector3D) -> Vector3D:
        """Interpolate position along the trajectory."""
        total_time = sum(trajectory)
        if t >= total_time:
            return final_pos
        if t <= 0:
            return initial_pos

        # Find which phase we're in
        current_time = 0.0
        position = initial_pos.x

        for i, phase_time in enumerate(trajectory):
            if current_time + phase_time >= t:
                # We're in this phase
                phase_t = t - current_time
                position = self._calculate_phase_position(
                    i, phase_t, initial_pos.x, final_pos.x, trajectory
                )
                break
            current_time += phase_time

        return Vector3D(position, initial_pos.y, initial_pos.z)

    def _calculate_phase_position(self, phase: int, t: float,
                                 initial_pos: float, final_pos: float,
                                 trajectory: List[float]) -> float:
        """Calculate position in a specific phase of the S-curve."""
        if phase == 0 or phase == 3 or phase == 4 or phase == 6:  # Jerk phases
            return self._jerk_phase_position(t, phase)
        elif phase == 1 or phase == 5:  # Constant acceleration phases
            return self._constant_accel_position(t, phase)
        elif phase == 2:  # Constant velocity phase
            return self._constant_vel_position(t, initial_pos, final_pos)
        else:
            return initial_pos

    def _jerk_phase_position(self, t: float, phase: int) -> float:
        """Calculate position during jerk phase."""
        if phase in [0, 6]:  # Acceleration increase/decrease
            return 0.5 * self.motion_params.max_jerk * t ** 2
        else:  # Deceleration phases
            v0 = self.motion_params.max_velocity
            a0 = self.motion_params.max_acceleration
            return v0 * t + 0.5 * a0 * t ** 2 - (1/6) * self.motion_params.max_jerk * t ** 3

    def _constant_accel_position(self, t: float, phase: int) -> float:
        """Calculate position during constant acceleration phase."""
        if phase == 1:  # Acceleration phase
            return self.motion_params.max_velocity * t + \
                   0.5 * self.motion_params.max_acceleration * t ** 2
        else:  # Deceleration phase
            v0 = self.motion_params.max_velocity
            a0 = self.motion_params.max_acceleration
            return v0 * t + 0.5 * a0 * t ** 2

    def _constant_vel_position(self, t: float, initial_pos: float,
                              final_pos: float) -> float:
        """Calculate position during constant velocity phase."""
        # Simplified constant velocity calculation
        distance = final_pos - initial_pos
        return initial_pos + distance * t / 10.0  # Assuming 10 second baseline


class BaseMotionController(ABC):
    """Base class for all motion controllers."""

    def __init__(self, stage_type: str, axis_limits: Dict[str, AxisLimits]):
        self.stage_type = stage_type
        self.axis_limits = axis_limits
        self.motion_params = MotionParameters(
            max_velocity=50.0,  # 50 microns/s default
            max_acceleration=500.0,  # 500 microns/s² default
            max_jerk=10000.0  # 10000 microns/s³ default
        )
        self.planner = SCurvePlanner(self.motion_params)

        # Current state
        self.current_position = self._get_home_position()
        self.current_velocity = Vector3D(0, 0, 0)
        self.current_acceleration = Vector3D(0, 0, 0)

        # Motion status
        self.is_moving = False
        self.target_position = None
        self.trajectory = []
        self.trajectory_start_time = 0.0
        self.emergency_stop_triggered = False

        # Status callbacks
        self.position_callbacks = []
        self.status_callbacks = []

    @abstractmethod
    def _get_home_position(self) -> StagePosition:
        """Get the home position for this stage."""
        pass

    @abstractmethod
    def get_bounding_box(self) -> BoundingBox:
        """Get the bounding box for collision detection."""
        pass

    def move_to_position(self, target: StagePosition,
                        velocity: Optional[float] = None,
                        acceleration: Optional[float] = None) -> bool:
        """
        Plan and execute move to target position.

        Args:
            target: Target position
            velocity: Optional maximum velocity override
            acceleration: Optional maximum acceleration override

        Returns:
            True if motion was started successfully, False otherwise
        """
        if self.emergency_stop_triggered:
            return False

        if not self._validate_position(target):
            return False

        if self.is_moving:
            return False  # Already moving

        # Update motion parameters if provided
        if velocity is not None:
            self.motion_params.max_velocity = velocity
        if acceleration is not None:
            self.motion_params.max_acceleration = acceleration

        # Plan trajectory for each axis
        self.target_position = target
        self.trajectory = self._plan_trajectory(self.current_position, target)

        if not self.trajectory:
            return False

        self.is_moving = True
        self.trajectory_start_time = time.time()

        # Notify status callbacks
        self._notify_status_change("moving", "Motion started")

        return True

    def _validate_position(self, position: StagePosition) -> bool:
        """Validate that position is within limits."""
        # Check linear axes
        for axis in ['x', 'y', 'z']:
            if axis in self.axis_limits:
                limits = self.axis_limits[axis]
                pos_value = getattr(position.position, axis)
                if not limits.check_position(pos_value):
                    return False

        # Check rotational axes if available
        if position.rotation:
            for axis in ['rx', 'ry', 'rz']:
                if axis in self.axis_limits:
                    limits = self.axis_limits[axis]
                    pos_value = getattr(position.rotation, axis)
                    if not limits.check_position(pos_value):
                        return False

        return True  # All checks passed

    def _plan_trajectory(self, initial: StagePosition,
                        final: StagePosition) -> List[float]:
        """Plan trajectory from initial to final position."""
        trajectory_times = []

        # Plan for each linear axis
        axes = ['x', 'y', 'z']
        for axis in axes:
            initial_val = getattr(initial.position, axis)
            final_val = getattr(final.position, axis)

            if abs(initial_val - final_val) > self.motion_params.position_tolerance:
                axis_trajectory = self.planner.plan_1d_move(initial_val, final_val)
                trajectory_times.extend(axis_trajectory)

        return trajectory_times

    def update(self):
        """Update the motion state (called at 1kHz frequency)."""
        if not self.is_moving:
            return

        current_time = time.time()
        elapsed_time = current_time - self.trajectory_start_time
        trajectory_duration = sum(self.trajectory) if self.trajectory else 0

        if elapsed_time >= trajectory_duration:
            # Motion complete
            self.current_position = self.target_position
            self.current_velocity = Vector3D(0, 0, 0)
            self.current_acceleration = Vector3D(0, 0, 0)
            self.is_moving = False
            self._notify_status_change("idle", "Motion complete")
        else:
            # Interpolate position along trajectory
            self.current_position = StagePosition(
                position=self.planner.interpolate_position(
                    self.trajectory, elapsed_time,
                    self.current_position.position,
                    self.target_position.position
                )
            )

        # Notify position callbacks
        self._notify_position_update()

    def emergency_stop(self):
        """Trigger emergency stop - halt all motion immediately."""
        self.emergency_stop_triggered = True
        self.is_moving = False
        self.current_velocity = Vector3D(0, 0, 0)
        self.current_acceleration = Vector3D(0, 0, 0)
        self.trajectory = []
        self._notify_status_change("emergency_stop", "Emergency stop triggered")

    def reset_emergency_stop(self):
        """Reset emergency stop - requires explicit reset."""
        self.emergency_stop_triggered = False
        self._notify_status_change("idle", "Emergency stop cleared")

    def get_current_state(self) -> MotionState:
        """Get the current motion state."""
        return MotionState(
            position=self.current_position,
            velocity=self.current_velocity,
            acceleration=self.current_acceleration,
            timestamp=time.time()
        )

    def add_position_callback(self, callback):
        """Add callback for position updates."""
        self.position_callbacks.append(callback)

    def add_status_callback(self, callback):
        """Add callback for status updates."""
        self.status_callbacks.append(callback)

    def _notify_position_update(self):
        """Notify all position callbacks."""
        state = self.get_current_state()
        for callback in self.position_callbacks:
            callback(state)

    def _notify_status_change(self, status: str, message: str):
        """Notify all status callbacks."""
        for callback in self.status_callbacks:
            callback(status, message)

    def set_motion_parameters(self, params: MotionParameters):
        """Update motion parameters."""
        self.motion_params = params
        self.planner = SCurvePlanner(params)