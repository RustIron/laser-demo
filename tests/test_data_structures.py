"""
Test core data structures for the motion control simulation.
"""

import pytest
import numpy as np
from core.data_structures import (
    Vector3D, Rotation3D, AxisLimits, StagePosition,
    BoundingBox, MotionState, StageType
)


class TestVector3D:
    """Test Vector3D operations."""

    def test_creation(self):
        """Test vector creation."""
        v = Vector3D(1, 2, 3)
        assert v.x == 1
        assert v.y == 2
        assert v.z == 3

    def test_addition(self):
        """Test vector addition."""
        v1 = Vector3D(1, 2, 3)
        v2 = Vector3D(4, 5, 6)
        result = v1 + v2
        assert result.x == 5
        assert result.y == 7
        assert result.z == 9

    def test_subtraction(self):
        """Test vector subtraction."""
        v1 = Vector3D(5, 7, 9)
        v2 = Vector3D(1, 2, 3)
        result = v1 - v2
        assert result.x == 4
        assert result.y == 5
        assert result.z == 6

    def test_multiplication(self):
        """Test scalar multiplication."""
        v = Vector3D(2, 3, 4)
        result = v * 2
        assert result.x == 4
        assert result.y == 6
        assert result.z == 8

    def test_division(self):
        """Test scalar division."""
        v = Vector3D(4, 6, 8)
        result = v / 2
        assert result.x == 2
        assert result.y == 3
        assert result.z == 4

    def test_magnitude(self):
        """Test magnitude calculation."""
        v = Vector3D(3, 4, 0)  # 3-4-5 triangle
        assert abs(v.magnitude() - 5.0) < 1e-10

    def test_normalize(self):
        """Test vector normalization."""
        v = Vector3D(3, 4, 0)
        normalized = v.normalize()
        assert abs(normalized.magnitude() - 1.0) < 1e-10
        # Check direction is preserved
        assert normalized.x / v.x == normalized.y / v.y

    def test_distance_to(self):
        """Test distance calculation."""
        v1 = Vector3D(0, 0, 0)
        v2 = Vector3D(3, 4, 0)
        assert abs(v1.distance_to(v2) - 5.0) < 1e-10

    def test_to_from_array(self):
        """Test array conversion."""
        original = Vector3D(1, 2, 3)
        arr = original.to_array()
        recovered = Vector3D.from_array(arr)
        assert recovered.x == original.x
        assert recovered.y == original.y
        assert recovered.z == original.z


class TestAxisLimits:
    """Test axis limits functionality."""

    def test_creation(self):
        """Test limits creation."""
        limits = AxisLimits(-100, 100, 1000, 5000)
        assert limits.min_position == -100
        assert limits.max_position == 100
        assert limits.max_velocity == 1000
        assert limits.max_acceleration == 5000

    def test_check_position(self):
        """Test position validation."""
        limits = AxisLimits(-100, 100, 1000, 5000)
        assert limits.check_position(0) == True
        assert limits.check_position(99) == True
        assert limits.check_position(101) == False
        assert limits.check_position(-101) == False

    def test_clamp_position(self):
        """Test position clamping."""
        limits = AxisLimits(-100, 100, 1000, 5000)
        assert limits.clamp_position(150) == 100
        assert limits.clamp_position(-150) == -100
        assert limits.clamp_position(50) == 50


class TestStagePosition:
    """Test stage position data structure."""

    def test_creation(self):
        """Test position creation."""
        position = Vector3D(10, 20, 30)
        rotation = Rotation3D(1, 2, 3)
        stage_pos = StagePosition(position, rotation)

        assert stage_pos.position.x == 10
        assert stage_pos.rotation.rx == 1

    def test_default_rotation(self):
        """Test default rotation creation."""
        position = Vector3D(10, 20, 30)
        stage_pos = StagePosition(position)

        assert stage_pos.rotation is not None
        assert stage_pos.rotation.rx == 0
        assert stage_pos.rotation.ry == 0
        assert stage_pos.rotation.rz == 0

    def test_to_from_dict(self):
        """Test dictionary conversion."""
        position = Vector3D(10, 20, 30)
        rotation = Rotation3D(1, 2, 3)
        stage_pos = StagePosition(position, rotation)

        data = stage_pos.to_dict()
        recovered = StagePosition.from_dict(data)

        assert recovered.position.x == stage_pos.position.x
        assert recovered.rotation.rx == stage_pos.rotation.rx


class TestBoundingBox:
    """Test bounding box functionality."""

    def test_creation(self):
        """Test bounding box creation."""
        min_corner = Vector3D(0, 0, 0)
        max_corner = Vector3D(10, 10, 10)
        bbox = BoundingBox(min_corner, max_corner)

        assert bbox.min_corner.x == 0
        assert bbox.max_corner.x == 10

    def test_contains_point(self):
        """Test point containment."""
        bbox = BoundingBox(
            Vector3D(0, 0, 0),
            Vector3D(10, 10, 10)
        )

        assert bbox.contains_point(Vector3D(5, 5, 5)) == True
        assert bbox.contains_point(Vector3D(10, 10, 10)) == True
        assert bbox.contains_point(Vector3D(-1, 5, 5)) == False
        assert bbox.contains_point(Vector3D(11, 5, 5)) == False

    def test_intersects(self):
        """Test box intersection."""
        box1 = BoundingBox(Vector3D(0, 0, 0), Vector3D(10, 10, 10))
        box2 = BoundingBox(Vector3D(5, 5, 5), Vector3D(15, 15, 15))
        box3 = BoundingBox(Vector3D(11, 11, 11), Vector3D(20, 20, 20))

        assert box1.intersects(box2) == True
        assert box1.intersects(box3) == False

    def test_auto_correction(self):
        """Test automatic min/max correction."""
        # Create box with coordinates in wrong order
        bbox = BoundingBox(
            Vector3D(10, 10, 10),
            Vector3D(0, 0, 0)
        )

        assert bbox.min_corner.x == 0
        assert bbox.max_corner.x == 10

    def test_properties(self):
        """Test bounding box properties."""
        bbox = BoundingBox(
            Vector3D(0, 0, 0),
            Vector3D(10, 10, 10)
        )

        center = bbox.center
        assert center.x == 5
        assert center.y == 5
        assert center.z == 5

        size = bbox.size
        assert size.x == 10
        assert size.y == 10
        assert size.z == 10


class TestMotionState:
    """Test motion state functionality."""

    def test_idle_creation(self):
        """Test idle state creation."""
        position = StagePosition(Vector3D(5, 10, 15))
        state = MotionState.idle(position, 1000.0)

        assert state.position.position.x == 5
        assert state.velocity.magnitude() == 0
        assert state.acceleration.magnitude() == 0
        assert state.timestamp == 1000.0

    def test_interpolation(self):
        """Test state interpolation."""
        pos1 = StagePosition(Vector3D(0, 0, 0), Rotation3D(0, 0, 0))
        state1 = MotionState(pos1, Vector3D(0, 0, 0), Vector3D(0, 0, 0), 0.0)

        pos2 = StagePosition(Vector3D(10, 20, 30), Rotation3D(90, 180, 270))
        state2 = MotionState(pos2, Vector3D(5, 10, 15), Vector3D(1, 2, 3), 10.0)

        # Midpoint interpolation
        mid = state1.interpolate(state2, 0.5)
        assert mid.position.position.x == 5
        assert mid.position.position.y == 10
        assert mid.position.position.z == 15
        assert mid.rotation.rx == 45
        assert mid.rotation.ry == 90
        assert mid.timestamp == 5.0


if __name__ == '__main__':
    pytest.main([__file__])