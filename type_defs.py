from typing import NamedTuple, Optional, Union, Dict, List, Set, Tuple
from dataclasses import dataclass
from enum import Enum, auto
import math

# --------------------------
# Basic Types
# --------------------------
Time = float
Id = int
Position = Tuple[float, float]

# --------------------------
# Core Data Structures (IMPROVED)
# --------------------------
class Coordinates(NamedTuple):
    """2D coordinates with enhanced math operations"""
    x: float
    y: float
    
    # Original methods
    def __add__(self, other):
        return Coordinates(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Coordinates(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Coordinates(self.x * scalar, self.y * scalar)
    
    def distance(self, other) -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __str__(self):
        return f"({self.x:.3f}, {self.y:.3f})"

    def as_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)
    
    # NEW: Added vector operations
    def magnitude(self) -> float:
        """Returns the vector length"""
        return math.sqrt(self.x**2 + self.y**2)
        
    def normalized(self) -> 'Coordinates':
        """Returns a unit vector"""
        mag = self.magnitude()
        return Coordinates(0, 0) if mag == 0 else Coordinates(self.x/mag, self.y/mag)

@dataclass
class Circle:
    """Circle with validation and geometric operations"""
    center: Coordinates
    radius: float
    
    # NEW: Runtime validation
    def __post_init__(self):
        if self.radius < 0:
            raise ValueError("Circle radius cannot be negative")
    
    # Original methods
    def contains(self, point: Coordinates) -> bool:
        return self.center.distance(point) <= self.radius
    
    def circumference(self) -> float:
        return 2 * math.pi * self.radius
    
    def area(self) -> float:
        return math.pi * self.radius ** 2
    
    def __str__(self):
        return f"Circle(center={self.center}, radius={self.radius:.3f})"

# --------------------------
# Robot State Types (IMPROVED)
# --------------------------
class SnapshotDetails(NamedTuple):
    """Complete robot state with type-safe faults"""
    pos: Coordinates
    state: 'RobotState'
    frozen: bool
    terminated: bool
    multiplicity: Optional[int]
    # CHANGED: Using proper Enum type
    fault_type: Optional['FaultType'] = None  # Now references FaultType enum
    velocity: Optional[Coordinates] = None
    orientation: Optional[float] = None
    
    # NEW: Added status check
    def is_operational(self) -> bool:
        """Check if robot can participate in swarm operations"""
        return not (self.frozen or self.terminated or self.fault_type)

# --------------------------
# Event System (IMPROVED)
# --------------------------
class Event(NamedTuple):
    """Enhanced event with validation"""
    time: float
    robot_id: int
    state: 'RobotState'
    priority: int = 0
    
    # NEW: Added input validation
    def __new__(cls, time: float, robot_id: int, state: 'RobotState', priority: int = 0):
        if time < 0:
            raise ValueError(f"Event time cannot be negative (got {time})")
        if priority < 0:
            raise ValueError(f"Priority cannot be negative (got {priority})")
        return super().__new__(cls, time, robot_id, state, priority)
    
    # Original comparison
    def __lt__(self, other):
        return (self.time, self.priority) < (other.time, other.priority)

# --------------------------
# Original Advanced Types (PRESERVED)
# --------------------------
class Orientation(NamedTuple):
    """Full orientation in 2D space"""
    translation: Coordinates
    rotation: float  # radians
    reflection: bool
    
    def transform(self, point: Coordinates) -> Coordinates:
        # Apply rotation matrix
        x = point.x * math.cos(self.rotation) - point.y * math.sin(self.rotation)
        y = point.x * math.sin(self.rotation) + point.y * math.cos(self.rotation)
        
        # Apply reflection if needed
        if self.reflection:
            x = -x
        
        # Apply translation
        return Coordinates(x + self.translation.x, y + self.translation.y)

class SwarmConfig(NamedTuple):
    """Complete configuration for swarm simulation"""
    num_robots: int
    visibility_radius: float
    algorithm: str
    fault_prob: float
    scheduler_type: str
    area_size: float
    speed_variance: float = 0.0
    comms_dropout: float = 0.0
    max_runtime: float = 60.0

# --------------------------
# Original Algorithm Types (PRESERVED)
# --------------------------
class SECResult(NamedTuple):
    """Result of smallest enclosing circle computation"""
    circle: Circle
    boundary_robots: List[Id]
    is_minimal: bool

class GatheringState(NamedTuple):
    """State for gathering algorithm"""
    centroid: Coordinates
    visible_robots: Set[Id]
    movement_vector: Coordinates

# --------------------------
# Original Visualization Types (PRESERVED)
# --------------------------
class FrameData(NamedTuple):
    """Complete data for visualization frame"""
    time: float
    positions: Dict[Id, Coordinates]
    states: Dict[Id, 'RobotState']
    faults: Dict[Id, Optional[str]]
    connections: Set[Tuple[Id, Id]]
    sec: Optional[Circle] = None
    
    # NEW: Added serialization
    def to_json(self) -> Dict:
        return {
            'time': self.time,
            'positions': {k: (v.x, v.y) for k,v in self.positions.items()},
            'states': {k: v.name for k,v in self.states.items()},
            'faults': self.faults,
            'connections': list(self.connections),
            'sec': {
                'center': (self.sec.center.x, self.sec.center.y),
                'radius': self.sec.radius
            } if self.sec else None
        }

# --------------------------
# Original RobotState Enum (PRESERVED)
# --------------------------
class RobotState(Enum):
    """Finite state machine states for robots"""
    WAIT = auto()
    LOOK = auto()
    COMPUTE = auto()
    MOVE = auto()
    FAULT = auto()
