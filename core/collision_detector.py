"""
Collision detection and safety interlock system for the test equipment.
"""

from typing import List, Dict, Tuple, Optional, Set
import time
from dataclasses import dataclass
from enum import Enum

from .data_structures import (
    Vector3D, StagePosition, BoundingBox, MotionState,
    StageType
)


class CollisionSeverity(Enum):
    """Severity levels for collision warnings."""
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class CollisionWarning:
    """Information about detected collision or safety violation."""
    stage1: str
    stage2: Optional[str]  # None for self-collision or limit violation
    severity: CollisionSeverity
    message: str
    timestamp: float
    position: Vector3D
    distance: Optional[float] = None  # Distance to collision if applicable


@dataclass
class SafetyZone:
    """Defined safety zones for restricted areas."""
    name: str
    bounding_box: BoundingBox
    allowed_stages: Set[str]  # Stages allowed in this zone
    min_clearance: float  # Minimum clearance required (microns)
    priority: int  # Higher priority zones override lower priority


class CollisionDetector:
    """Advanced collision detection with predictive capabilities."""

    def __init__(self):
        self.stages: Dict[str, any] = {}  # Registered stage controllers
        self.safety_zones: List[SafetyZone] = []
        self.warning_callbacks = []
        self.emergency_stop_callbacks = []

        # Safety parameters
        self.min_clearance = 1000.0  # 1mm minimum clearance between stages
        self.prediction_time = 2.0  # Predict collisions 2 seconds ahead
        self.enabled = True

        # Tracking
        self.current_warnings: List[CollisionWarning] = []
        self.collision_history: List[CollisionWarning] = []
        self.max_history_size = 1000

        # Default safety zones initialization
        self._initialize_default_safety_zones()

    def register_stage(self, name: str, stage_controller: any):
        """Register a stage controller for collision monitoring."""
        self.stages[name] = stage_controller

        # Add position update callback
        stage_controller.add_position_callback(
            lambda state, n=name: self._on_position_update(n, state)
        )

    def add_safety_zone(self, zone: SafetyZone):
        """Add a custom safety zone."""
        self.safety_zones.append(zone)
        # Sort by priority
        self.safety_zones.sort(key=lambda z: z.priority, reverse=True)

    def add_warning_callback(self, callback):
        """Add callback for collision warnings."""
        self.warning_callbacks.append(callback)

    def add_emergency_stop_callback(self, callback):
        """Add callback for emergency stop triggers."""
        self.emergency_stop_callbacks.append(callback)

    def check_all_collisions(self) -> List[CollisionWarning]:
        """Perform comprehensive collision checking."""
        warnings = []

        if not self.enabled:
            return warnings

        # Check stage-to-stage collisions
        warnings.extend(self._check_stage_collisions())

        # Check safety zone violations
        warnings.extend(self._check_safety_zone_violations())

        # Check predictive collisions
        warnings.extend(self._check_predictive_collisions())

        # Check equipment limits
        warnings.extend(self._check_equipment_limits())

        # Track new warnings
        for warning in warnings:
            self._handle_new_warning(warning)

        return warnings

    def _check_stage_collisions(self) -> List[CollisionWarning]:
        """Check for actual collisions between stages."""
        warnings = []

        stage_list = list(self.stages.items())
        for i, (name1, stage1) in enumerate(stage_list):
            bbox1 = stage1.get_bounding_box()

            # Check collision with other stages
            for name2, stage2 in stage_list[i+1:]:
                bbox2 = stage2.get_bounding_box()

                # Calculate distance between bounding boxes
                distance = self._calculate_box_distance(bbox1, bbox2)

                if distance < self.min_clearance:
                    severity = self._determine_collision_severity(distance)

                    # Get collision point (center of closest approach)
                    collision_point = self._get_closest_point_between_boxes(bbox1, bbox2)

                    warning = CollisionWarning(
                        stage1=name1,
                        stage2=name2,
                        severity=severity,
                        message=f"Stage collision: {name1} and {name2}",
                        timestamp=time.time(),
                        position=collision_point,
                        distance=distance
                    )

                    warnings.append(warning)

                    # Trigger emergency stop for critical collisions
                    if severity == CollisionSeverity.EMERGENCY_STOP:
                        self._trigger_emergency_stop(warning)

        return warnings

    def _check_safety_zone_violations(self) -> List[CollisionWarning]:
        """Check for violations of defined safety zones."""
        warnings = []

        for zone in self.safety_zones:
            for name, stage in self.stages.items():
                if name in zone.allowed_stages:
                    continue  # This stage is allowed in this zone

                bbox = stage.get_bounding_box()

                if self._box_intersects_zone(bbox, zone.bounding_box):
                    warning = CollisionWarning(
                        stage1=name,
                        stage2=f"safety_zone:{zone.name}",
                        severity=CollisionSeverity.WARNING,
                        message=f"Stage {name} violated safety zone '{zone.name}'",
                        timestamp=time.time(),
                        position=zone.bounding_box.center,
                        distance=0.0
                    )
                    warnings.append(warning)

        return warnings

    def _check_predictive_collisions(self) -> List[CollisionWarning]:
        """Predict future collisions based on current velocities."""
        warnings = []

        for name, stage in self.stages.items():
            if not stage.is_moving:
                continue

            # Predict future position
            future_time = time.time() + self.prediction_time
            future_state = self._predict_future_state(stage, self.prediction_time)

            # Create temporary bounding box for future position
            current_bbox = stage.get_bounding_box()
            displacement = future_state.position.position - stage.current_position.position
            future_bbox = BoundingBox(
                min_corner=current_bbox.min_corner + displacement,
                max_corner=current_bbox.max_corner + displacement
            )

            # Check future bbox against other stages
            for other_name, other_stage in self.stages.items():
                if other_name == name:
                    continue

                other_bbox = other_stage.get_bounding_box()
                future_distance = self._calculate_box_distance(future_bbox, other_bbox)

                if future_distance < self.min_clearance * 2:  # Give more warning time
                    warning = CollisionWarning(
                        stage1=name,
                        stage2=other_name,
                        severity=CollisionSeverity.WARNING,
                        message=f"Predicted collision in {self.prediction_time}s: {name} and {other_name}",
                        timestamp=time.time(),
                        position=future_state.position.position,
                        distance=future_distance
                    )
                    warnings.append(warning)

        return warnings

    def _check_equipment_limits(self) -> List[CollisionWarning]:
        """Check for equipment limit violations."""
        warnings = []

        for name, stage in self.stages.items():
            current_state = stage.get_current_state()
            pos = current_state.position.position

            # Check velocity limits
            vel = current_state.velocity
            if vel.magnitude() > stage.motion_params.max_velocity * 0.95:
                warning = CollisionWarning(
                    stage1=name,
                    stage2=None,
                    severity=CollisionSeverity.WARNING,
                    message=f"Stage {name} approaching velocity limit: {vel.magnitude():.1f} microns/s",
                    timestamp=time.time(),
                    position=pos
                )
                warnings.append(warning)

            # Check if probe is pressing too hard (for probe stages)
            if isinstance(stage, type(self.stages.get('probe_stage'))):
                if hasattr(stage, 'current_force') and stage.current_force > stage.press_force_limit * 0.9:
                    warning = CollisionWarning(
                        stage1=name,
                        stage2=None,
                        severity=CollisionSeverity.CRITICAL,
                        message=f"Probe force limit approaching: {stage.current_force:.1f}",
                        timestamp=time.time(),
                        position=pos
                    )
                    warnings.append(warning)

        return warnings

    def _calculate_box_distance(self, bbox1: BoundingBox, bbox2: BoundingBox) -> float:
        """Calculate minimum distance between two bounding boxes."""
        # If boxes intersect, distance is 0
        if bbox1.intersects(bbox2):
            return 0.0

        # Calculate distance for each axis
        dx = max(0, max(bbox1.min_corner.x - bbox2.max_corner.x,
                        bbox2.min_corner.x - bbox1.max_corner.x))
        dy = max(0, max(bbox1.min_corner.y - bbox2.max_corner.y,
                        bbox2.min_corner.y - bbox1.max_corner.y))
        dz = max(0, max(bbox1.min_corner.z - bbox2.max_corner.z,
                        bbox2.min_corner.z - bbox1.max_corner.z))

        # Euclidean distance
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def _get_closest_point_between_boxes(self, bbox1: BoundingBox,
                                        bbox2: BoundingBox) -> Vector3D:
        """Get the point of closest approach between two boxes."""
        # For simplicity, return the midpoint between the two boxes
        center1 = bbox1.center
        center2 = bbox2.center
        return (center1 + center2) / 2

    def _determine_collision_severity(self, distance: float) -> CollisionSeverity:
        """Determine collision severity based on distance."""
        if distance <= 0:
            return CollisionSeverity.EMERGENCY_STOP
        elif distance < self.min_clearance * 0.25:
            return CollisionSeverity.CRITICAL
        elif distance < self.min_clearance * 0.5:
            return CollisionSeverity.WARNING
        else:
            return CollisionSeverity.INFO

    def _box_intersects_zone(self, bbox: BoundingBox, zone_bbox: BoundingBox) -> bool:
        """Check if a bounding box intersects with a safety zone."""
        return bbox.intersects(zone_bbox)

    def _predict_future_state(self, stage, time_ahead: float) -> MotionState:
        """Predict future state of a moving stage."""
        current_state = stage.get_current_state()

        if not stage.is_moving or time_ahead <= 0:
            return current_state

        # Simple prediction based on current velocity
        # In a real system, this would use the actual trajectory
        displacement = current_state.velocity * time_ahead
        future_position = StagePosition(
            position=current_state.position.position + displacement,
            rotation=current_state.position.rotation
        )

        return MotionState(
            position=future_position,
            velocity=current_state.velocity,  # Assuming constant velocity
            acceleration=Vector3D(0, 0, 0),  # No acceleration in simple prediction
            timestamp=current_state.timestamp + time_ahead
        )

    def _handle_new_warning(self, warning: CollisionWarning):
        """Handle new collision warning."""
        # Check if this is a new warning (not a duplicate)
        is_duplicate = any(
            w.stage1 == warning.stage1 and
            w.stage2 == warning.stage2 and
            w.severity == warning.severity and
            (time.time() - w.timestamp) < 1.0
            for w in self.current_warnings
        )

        if not is_duplicate:
            self.current_warnings.append(warning)
            self.collision_history.append(warning)

            # Limit history size
            if len(self.collision_history) > self.max_history_size:
                self.collision_history.pop(0)

            # Notify callbacks
            for callback in self.warning_callbacks:
                callback(warning)

    def _trigger_emergency_stop(self, warning: CollisionWarning):
        """Trigger emergency stop for critical collisions."""
        # Stop all stages
        for stage in self.stages.values():
            stage.emergency_stop()

        # Notify emergency stop callbacks
        for callback in self.emergency_stop_callbacks:
            callback(warning)

    def _on_position_update(self, stage_name: str, state: MotionState):
        """Callback for stage position updates."""
        # Clear old warnings for this stage
        self.current_warnings = [
            w for w in self.current_warnings
            if w.stage1 != stage_name and
            (time.time() - w.timestamp) > 2.0  # Keep warnings for 2 seconds
        ]

        # Check for new collisions
        warnings = self.check_all_collisions()

    def _initialize_default_safety_zones(self):
        """Initialize default safety zones."""
        # Chip loading zone - only gantry allowed
        chip_loading_zone = SafetyZone(
            name="chip_loading",
            bounding_box=BoundingBox(
                min_corner=Vector3D(-150000, -150000, -10000),
                max_corner=Vector3D(150000, 150000, 60000)
            ),
            allowed_stages={"three_axis_gantry"},
            min_clearance=2000.0,  # 2mm
            priority=1
        )

        # Probe zone - only probe stage allowed when engaged
        probe_zone = SafetyZone(
            name="probe_area",
            bounding_box=BoundingBox(
                min_corner=Vector3D(-30000, -30000, -15000),
                max_corner=Vector3D(30000, 30000, 15000)
            ),
            allowed_stages={"six_axis", "probe_stage"},
            min_clearance=500.0,  # 0.5mm
            priority=3  # High priority
        )

        self.safety_zones.extend([chip_loading_zone, probe_zone])

    def enable(self):
        """Enable collision detection."""
        self.enabled = True

    def disable(self):
        """Disable collision detection."""
        self.enabled = False

    def get_current_warnings(self) -> List[CollisionWarning]:
        """Get all current collision warnings."""
        return self.current_warnings.copy()

    def get_collision_history(self, limit: int = 100) -> List[CollisionWarning]:
        """Get recent collision history."""
        return self.collision_history[-limit:]

    def clear_warnings(self):
        """Clear all current warnings."""
        self.current_warnings.clear()