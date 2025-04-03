from typing import NamedTuple, TypedDict, NewType
from dataclasses import dataclass

# ===== ORIGINAL CODE (UNCHANGED) =====
Time = float
Id = int

class Coordinates(NamedTuple):
    """
    x: float \n
    y: float
    """
    x: float
    y: float

    def __str__(self):
        return f"({float(self.x)}, {float(self.y)})"

class Circle(NamedTuple):
    """
    center: Coordinates \n
    radius: float
    """
    center: Coordinates
    radius: float

    def __str__(self):
        return f"Center: {self.center} ; radius: {float(self.radius)}"

class SnapshotDetails(NamedTuple):
    """
    pos: Coordinates \n
    state: float \n
    frozen: bool \n
    terminated: bool \n
    multiplicity: int | None
    """
    pos: Coordinates
    state: str
    frozen: bool
    terminated: bool
    multiplicity: int | None

class Event(NamedTuple):
    """
    time: float | Time \n
    id: int \n
    state: str
    """
    time: float
    id: int
    state: str

class Orientation(NamedTuple):
    """
    translation = float \n
    rotation = float \n
    reflection = float
    """
    translation = float
    rotation = float
    reflection = float

# ===== NEW TYPE DEFINITIONS =====
class FaultConfig(NamedTuple):
    """Configuration for robot faults
    type: str  # 'crash', 'byzantine', or 'delay'
    probability: float  # 0.0 to 1.0
    severity: float = 1.0  # Intensity of fault effect
    """
    type: str
    probability: float
    severity: float = 1.0

class VisibilityConfig(NamedTuple):
    """Robot visibility parameters
    range: float  # Maximum visibility distance
    angle: float  # Field of view in radians
    obstructed: bool  # Whether obstructions are enabled
    """
    range: float
    angle: float
    obstructed: bool

# Type aliases
RobotID = NewType('RobotID', int)
VisibilityGraph = dict[RobotID, list[RobotID]]  # Maps robot ID to visible neighbors
MovementVector = NewType('MovementVector', tuple[float, float])
