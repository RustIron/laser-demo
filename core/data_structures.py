"""
Core data structures for the motion control simulation environment.
"""

from dataclasses import dataclass, fields
from typing import Optional, Tuple, Dict, Any
import numpy as np
from enum import Enum


class StageType(Enum):
    """Enumeration of different stage types."""
    SIX_AXIS = "six_axis"
    THREE_AXIS_GANTRY = "three_axis_gantry"
    PROBE_STAGE = "probe_stage"


@dataclass
class Vector3D:
    """3D vector with micron precision."""
    x: float  # Position in microns
    y: float  # Position in microns
    z: float  # Position in microns

    def __add__(self, other):
        if isinstance(other, Vector3D):
            return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
        raise TypeError(f"Cannot add Vector3D and {type(other)}")

    def __sub__(self, other):
        if isinstance(other, Vector3D):
            return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
        raise TypeError(f"Cannot subtract Vector3D and {type(other)}")

    def __mul__(self, scalar):
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar):
        if scalar == 0:
            raise ValueError("Division by zero")
        return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)

    def magnitude(self) -> float:
        """Calculate the magnitude of the vector."""
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        """Return a normalized copy of the vector."""
        mag = self.magnitude()
        if mag == 0:
            return Vector3D(0, 0, 0)
        return self / mag

    def distance_to(self, other) -> float:
        """Calculate distance to another Vector3D."""
        return (self - other).magnitude()

    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_array(cls, array: np.ndarray) -> 'Vector3D':
        """Create from numpy array."""
        if len(array) != 3:
            raise ValueError("Array must have exactly 3 elements")
        return cls(float(array[0]), float(array[1]), float(array[2]))

    def __repr__(self) -> str:
        return f"Vector3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"


@dataclass
class Rotation3D:
    """3D rotation for angular positions (in degrees)."""
    rx: float = 0.0  # Rotation around X axis (degrees)
    ry: float = 0.0  # Rotation around Y axis (degrees)
    rz: float = 0.0  # Rotation around Z axis (degrees)

    def to_radians(self) -> Tuple[float, float, float]:
        """Convert all rotations to radians."""
        return (np.radians(self.rx), np.radians(self.ry), np.radians(self.rz))

    def __repr__(self) -> str:
        return f"Rotation3D(rx={self.rx:.3f}°, ry={self.ry:.3f}°, rz={self.rz:.3f}°)"


@dataclass
class AxisLimits:
    """Motion limits for individual axes."""
    min_position: float  # Minimum position (microns)
    max_position: float  # Maximum position (microns)
    max_velocity: float  # Maximum velocity (microns/s)
    max_acceleration: float  # Maximum acceleration (microns/s²)

    def check_position(self, position: float) -> bool:
        """Check if position is within limits."""
        return self.min_position <= position <= self.max_position

    def check_velocity(self, velocity: float) -> bool:
        """Check if velocity is within limits."""
        return abs(velocity) <= self.max_velocity

    def check_acceleration(self, acceleration: float) -> bool:
        """Check if acceleration is within limits."""
        return abs(acceleration) <= self.max_acceleration

    def clamp_position(self, position: float) -> float:
        """Clamp position to valid range."""
        return max(self.min_position, min(self.max_position, position))

    def clamp_velocity(self, velocity: float) -> float:
        """Clamp velocity to valid range."""
        return max(-self.max_velocity, min(self.max_velocity, velocity))

    def clamp_acceleration(self, acceleration: float) -> float:
        """Clamp acceleration to valid range."""
        return max(-self.max_acceleration, min(self.max_acceleration, acceleration))


@dataclass
class StagePosition:
    """Complete position information for a stage."""
    position: Vector3D
    rotation: Optional[Rotation3D] = None

    def __post_init__(self):
        if self.rotation is None:
            self.rotation = Rotation3D()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        result = {
            'x': self.position.x,
            'y': self.position.y,
            'z': self.position.z
        }
        if self.rotation is not None:
            result.update({
                'rx': self.rotation.rx,
                'ry': self.rotation.ry,
                'rz': self.rotation.rz
            })
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'StagePosition':
        """Create from dictionary representation."""
        position = Vector3D(
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            z=float(data.get('z', 0.0))
        )

        rotation = Rotation3D(
            rx=float(data.get('rx', 0.0)),
            ry=float(data.get('ry', 0.0)),
            rz=float(data.get('rz', 0.0))
        )

        return cls(position, rotation)

    def distance_to(self, other: 'StagePosition') -> float:
        """Calculate the distance to another position."""
        return self.position.distance_to(other.position)


@dataclass
class BoundingBox:
    """Axis-aligned bounding box for collision detection."""
    min_corner: Vector3D
    max_corner: Vector3D

    def __post_init__(self):
        # Ensure min_corner is actually the minimum
        self.min_corner = Vector3D(
            min(self.min_corner.x, self.max_corner.x),
            min(self.min_corner.y, self.max_corner.y),
            min(self.min_corner.z, self.max_corner.z)
        )
        # Ensure max_corner is actually the maximum
        self.max_corner = Vector3D(
            max(self.min_corner.x, self.max_corner.x),
            max(self.min_corner.y, self.max_corner.y),
            max(self.min_corner.z, self.max_corner.z)
        )

    def contains_point(self, point: Vector3D) -> bool:
        """Check if the bounding box contains a point."""
        return (self.min_corner.x <= point.x <= self.max_corner.x and
                self.min_corner.y <= point.y <= self.max_corner.y and
                self.min_corner.z <= point.z <= self.max_corner.z)

    def intersects(self, other: 'BoundingBox') -> bool:
        """Check if this bounding box intersects with another."""
        return (self.min_corner.x <= other.max_corner.x and
                self.max_corner.x >= other.min_corner.x and
                self.min_corner.y <= other.max_corner.y and
                self.max_corner.y >= other.min_corner.y and
                self.min_corner.z <= other.max_corner.z and
                self.max_corner.z >= other.min_corner.z)

    @property
    def center(self) -> Vector3D:
        """Get the center point of the bounding box."""
        return (self.min_corner + self.max_corner) / 2

    @property
    def size(self) -> Vector3D:
        """Get the size of the bounding box."""
        return self.max_corner - self.min_corner


@dataclass
class MotionState:
    """Current motion state of a stage."""
    position: StagePosition
    velocity: Vector3D
    acceleration: Vector3D
    timestamp: float

    @classmethod
    def idle(cls, position: StagePosition, timestamp: float = 0.0) -> 'MotionState':
        """Create an idle motion state."""
        return cls(
            position=position,
            velocity=Vector3D(0, 0, 0),
            acceleration=Vector3D(0, 0, 0),
            timestamp=timestamp
        )

    def interpolate(self, other: 'MotionState', t: float) -> 'MotionState':
        """Interpolate between this state and another state."""
        if not 0 <= t <= 1:
            raise ValueError("Interpolation parameter t must be between 0 and 1")

        position = StagePosition(
            position=Vector3D(
                x=self.position.position.x + t * (other.position.position.x - self.position.position.x),
                y=self.position.position.y + t * (other.position.position.y - self.position.position.y),
                z=self.position.position.z + t * (other.position.position.z - self.position.position.z)
            ),
            rotation=Rotation3D(
                rx=self.position.rotation.rx + t * (other.position.rotation.rx - self.position.rotation.rx),
                ry=self.position.rotation.ry + t * (other.position.rotation.ry - self.position.rotation.ry),
                rz=self.position.rotation.rz + t * (other.position.rotation.rz - self.position.rotation.rz)
            ) if self.position.rotation else None
        )

        velocity = Vector3D(
            x=self.velocity.x + t * (other.velocity.x - self.velocity.x),
            y=self.velocity.y + t * (other.velocity.y - self.velocity.y),
            z=self.velocity.z + t * (other.velocity.z - self.velocity.z)
        )

        acceleration = Vector3D(
            x=self.acceleration.x + t * (other.acceleration.x - self.acceleration.x),
            y=self.acceleration.y + t * (other.acceleration.y - self.acceleration.y),
            z=self.acceleration.z + t * (other.acceleration.z - self.acceleration.z)
        )

        timestamp = self.timestamp + t * (other.timestamp - self.timestamp)

        return MotionState(position, velocity, acceleration, timestamp)