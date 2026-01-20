"""
Specific stage controller implementations for the laser diode test equipment.
"""

from typing import Dict, Optional
import time

from .motion_controller import BaseMotionController, MotionParameters
from .data_structures import (
    Vector3D, StagePosition, AxisLimits, BoundingBox,
    StageType
)


class SixAxisStageController(BaseMotionController):
    """Controller for the 6-axis positioning stage (X, Y, Z, Rx, Ry, Rz)."""

    def __init__(self):
        # Define axis limits in microns for position and degrees for rotation
        axis_limits = {
            'x': AxisLimits(min_position=-50000, max_position=50000,     # ±50mm
                          max_velocity=100000, max_acceleration=500000),  # 100mm/s, 500mm/s²
            'y': AxisLimits(min_position=-50000, max_position=50000,     # ±50mm
                          max_velocity=100000, max_acceleration=500000),  # 100mm/s, 500mm/s²
            'z': AxisLimits(min_position=-10000, max_position=10000,     # ±10mm
                          max_velocity=50000, max_acceleration=250000),   # 50mm/s, 250mm/s²
            'rx': AxisLimits(min_position=-180, max_position=180,        # ±180 degrees
                            max_velocity=360, max_acceleration=1800),     # 360°/s, 1800°/s²
            'ry': AxisLimits(min_position=-180, max_position=180,        # ±180 degrees
                            max_velocity=360, max_acceleration=1800),     # 360°/s, 1800°/s²
            'rz': AxisLimits(min_position=-360, max_position=360,        # ±360 degrees
                            max_velocity=720, max_acceleration=3600),     # 720°/s, 3600°/s²
        }

        super().__init__("six_axis", axis_limits)

        # Set specific motion parameters for 6-axis stage
        self.motion_params = MotionParameters(
            max_velocity=100000,  # 100mm/s
            max_acceleration=500000,  # 500mm/s²
            max_jerk=10000000,  # 10000mm/s³
            position_tolerance=0.1,  # 0.1 micron
            velocity_tolerance=10.0  # 10 microns/s
        )
        self.planner = SCurvePlanner(self.motion_params)

        # Physical dimensions for collision detection (in microns)
        self.stage_size = Vector3D(100000, 100000, 20000)  # 100x100x20mm
        self.probe_length = 30000  # 30mm probe length

    def _get_home_position(self) -> StagePosition:
        """Get the home position for the 6-axis stage."""
        return StagePosition(
            position=Vector3D(0, 0, 0),
            rotation=None  # Zero rotation by default
        )

    def get_bounding_box(self) -> BoundingBox:
        """Get the bounding box for collision detection."""
        current_pos = self.current_position.position
        half_size = self.stage_size / 2

        return BoundingBox(
            min_corner=current_pos - half_size,
            max_corner=current_pos + half_size
        )

    def get_probe_tip_position(self) -> Vector3D:
        """Calculate the current probe tip position considering rotation."""
        base_pos = self.current_position.position
        rotation = self.current_position.rotation

        if not rotation or (rotation.rx == 0 and rotation.ry == 0 and rotation.rz == 0):
            # No rotation, probe extends straight down
            return Vector3D(base_pos.x, base_pos.y, base_pos.z - self.probe_length)

        # Apply rotation to probe vector (simplified)
        # In a real implementation, this would use full 3D rotation matrices
        probe_tip = Vector3D(0, 0, -self.probe_length)

        # Simple rotation around X axis
        if rotation.rx != 0:
            angle_rad = np.radians(rotation.rx)
            new_y = probe_tip.y * np.cos(angle_rad) - probe_tip.z * np.sin(angle_rad)
            new_z = probe_tip.y * np.sin(angle_rad) + probe_tip.z * np.cos(angle_rad)
            probe_tip.y = new_y
            probe_tip.z = new_z

        # Simple rotation around Y axis
        if rotation.ry != 0:
            angle_rad = np.radians(rotation.ry)
            new_x = probe_tip.x * np.cos(angle_rad) + probe_tip.z * np.sin(angle_rad)
            new_z = -probe_tip.x * np.sin(angle_rad) + probe_tip.z * np.cos(angle_rad)
            probe_tip.x = new_x
            probe_tip.z = new_z

        # Simple rotation around Z axis
        if rotation.rz != 0:
            angle_rad = np.radians(rotation.rz)
            new_x = probe_tip.x * np.cos(angle_rad) - probe_tip.y * np.sin(angle_rad)
            new_y = probe_tip.x * np.sin(angle_rad) + probe_tip.y * np.cos(angle_rad)
            probe_tip.x = new_x
            probe_tip.y = new_y

        return base_pos + probe_tip


class ThreeAxisGantryController(BaseMotionController):
    """Controller for the 3-axis gantry system (X, Y, Z)."""

    def __init__(self):
        # Define axis limits for chip loading gantry
        axis_limits = {
            'x': AxisLimits(min_position=-100000, max_position=100000,   # ±100mm
                          max_velocity=200000, max_acceleration=1000000),  # 200mm/s, 1000mm/s²
            'y': AxisLimits(min_position=-100000, max_position=100000,   # ±100mm
                          max_velocity=200000, max_acceleration=1000000),  # 200mm/s, 1000mm/s²
            'z': AxisLimits(min_position=-5000, max_position=50000,      # -5mm to 50mm
                          max_velocity=100000, max_acceleration=500000),   # 100mm/s, 500mm/s²
        }

        super().__init__("three_axis_gantry", axis_limits)

        # Set specific motion parameters for gantry
        self.motion_params = MotionParameters(
            max_velocity=200000,  # 200mm/s (faster for loading)
            max_acceleration=1000000,  # 1000mm/s²
            max_jerk=20000000,  # 20000mm/s³
            position_tolerance=0.5,  # 0.5 micron (less precision needed)
            velocity_tolerance=20.0  # 20 microns/s
        )
        self.planner = SCurvePlanner(self.motion_params)

        # Physical dimensions
        self.gantry_size = Vector3D(120000, 120000, 30000)  # 120x120x30mm
        self.gripper_height = 10000  # 10mm gripper height
        self.has_chip = False  # Status flag for loaded chip

    def _get_home_position(self) -> StagePosition:
        """Get the home position for the gantry."""
        return StagePosition(
            position=Vector3D(0, 0, 30000),  # Start at safe height
            rotation=None
        )

    def get_bounding_box(self) -> BoundingBox:
        """Get the bounding box for collision detection."""
        current_pos = self.current_position.position
        half_size = self.gantry_size / 2

        return BoundingBox(
            min_corner=current_pos - half_size,
            max_corner=current_pos + half_size
        )

    def set_chip_loaded(self, loaded: bool):
        """Set the chip loaded status."""
        self.has_chip = loaded
        self._notify_status_change(
            "chip_loaded" if loaded else "chip_empty",
            f"Chip {'loaded' if loaded else 'removed'}"
        )

    def get_chip_position(self) -> Optional[Vector3D]:
        """Get the position of the gripped chip."""
        if not self.has_chip:
            return None

        # Chip is at the bottom of the gripper
        return Vector3D(
            self.current_position.position.x,
            self.current_position.position.y,
            self.current_position.position.z - self.gripper_height
        )


class ProbeStageController(BaseMotionController):
    """Controller for the probe stage with press/unpress functionality."""

    def __init__(self):
        # Define axis limits for probe stage (Z-axis for press/unpress)
        axis_limits = {
            'x': AxisLimits(min_position=-20000, max_position=20000,     # ±20mm
                          max_velocity=50000, max_acceleration=250000),    # 50mm/s, 250mm/s²
            'y': AxisLimits(min_position=-20000, max_position=20000,     # ±20mm
                          max_velocity=50000, max_acceleration=250000),    # 50mm/s, 250mm/s²
            'z': AxisLimits(min_position=-10000, max_position=10000,     # ±10mm (press range)
                          max_velocity=20000, max_acceleration=100000),    # 20mm/s, 100mm/s²
        }

        super().__init__("probe_stage", axis_limits)

        # Set specific motion parameters for probe stage
        self.motion_params = MotionParameters(
            max_velocity=50000,    # 50mm/s (controlled for probing)
            max_acceleration=250000,  # 250mm/s²
            max_jerk=5000000,     # 5000mm/s³
            position_tolerance=0.01,  # 0.01 micron (high precision)
            velocity_tolerance=1.0    # 1 micron/s
        )
        self.planner = SCurvePlanner(self.motion_params)

        # Probe-specific parameters
        self.probe_height = 5000  # 5mm probe height
        self.press_force_limit = 1000  # Force limit in arbitrary units
        self.is_pressed = False
        self.current_force = 0.0

    def _get_home_position(self) -> StagePosition:
        """Get the home position for the probe stage."""
        return StagePosition(
            position=Vector3D(0, 0, self.probe_height),  # Raised position
            rotation=None
        )

    def get_bounding_box(self) -> BoundingBox:
        """Get the bounding box for collision detection."""
        current_pos = self.current_position.position

        return BoundingBox(
            min_corner=current_pos - Vector3D(5000, 5000, self.probe_height),
            max_corner=current_pos + Vector3D(5000, 5000, 0)
        )

    def press_probe(self, target_z: float, max_force: float = None) -> bool:
        """
        Press the probe to a target Z position with force control.

        Args:
            target_z: Target Z position for pressing
            max_force: Maximum allowed force (optional)

        Returns:
            True if press operation started successfully
        """
        if max_force is not None:
            self.press_force_limit = max_force

        # Check if we're already pressed
        if self.is_pressed:
            return False

        # Calculate press position
        current_pos = self.current_position.position
        press_position = StagePosition(
            position=Vector3D(current_pos.x, current_pos.y, target_z),
            rotation=None
        )

        # Use slower velocity for precise pressing
        success = self.move_to_position(
            press_position,
            velocity=10000,  # 10mm/s for precision
            acceleration=50000  # 50mm/s²
        )

        if success:
            self.is_pressed = True
            self._notify_status_change("pressing", f"Pressing probe to Z={target_z}")

        return success

    def retract_probe(self) -> bool:
        """
        Retract the probe to the home (raised) position.

        Returns:
            True if retraction started successfully
        """
        home_pos = self._get_home_position()
        success = self.move_to_position(home_pos)

        if success:
            self.is_pressed = False
            self.current_force = 0.0
            self._notify_status_change("retracting", "Probe retracted")

        return success

    def update_probe_force(self):
        """Update the simulated probe force based on position."""
        if self.is_pressed and self.current_position:
            # Simulate force based on deflection from neutral
            neutral_z = self.probe_height
            current_z = self.current_position.position.z
            deflection = neutral_z - current_z

            # Simple spring model: F = k * deflection
            spring_constant = 0.1  # Arbitrary units
            self.current_force = min(deflection * spring_constant, self.press_force_limit)

            # Check force limit
            if self.current_force >= self.press_force_limit:
                self.emergency_stop()
                self._notify_status_change(
                    "force_limit_exceeded",
                    f"Force limit exceeded: {self.current_force:.1f} > {self.press_force_limit}"
                )

    def update(self):
        """Override update to include force calculations."""
        super().update()
        self.update_probe_force()


# Import numpy for rotation calculations
import numpy as np
from .motion_controller import SCurvePlanner