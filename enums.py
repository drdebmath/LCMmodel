from enum import Enum, auto
from typing import Dict, List

class SwarmEnum(Enum):
    """Base enum class with common utilities for all swarm enums"""
    @classmethod
    def names(cls) -> List[str]:
        """Get all enum member names as strings"""
        return [e.name for e in cls]
        
    @classmethod 
    def values(cls) -> List:
        """Get all enum values"""
        return [e.value for e in cls]

    def __str__(self):
        """Default string representation (override in child classes if needed)"""
        return self.name.replace('_', ' ').title()

# --- Core Enums ---
class RobotState(SwarmEnum, str):
    """Enhanced robot state machine with next-state transitions"""
    LOOK = "LOOK"
    MOVE = "MOVE"
    WAIT = "WAIT"
    TERMINATED = "TERMINATED"

    def next_state(self) -> 'RobotState':
        """State transition logic"""
        transitions = {
            RobotState.LOOK: RobotState.MOVE,
            RobotState.MOVE: RobotState.WAIT,
            RobotState.WAIT: RobotState.LOOK,
            RobotState.TERMINATED: RobotState.TERMINATED
        }
        return transitions[self]

class SchedulerType(SwarmEnum):
    """Scheduler implementations"""
    ASYNC = "Async"
    SYNC = "Sync"
    SEMI_SYNC = "Semi-Sync"

class DistributionType(SwarmEnum):
    """Probability distributions for timing"""
    EXPONENTIAL = "Exponential"
    UNIFORM = "Uniform"
    NORMAL = "Normal"

# --- Enhanced Enums ---
class Algorithm(SwarmEnum):
    """Swarm algorithms with metadata support"""
    GATHERING = auto()
    SEC = auto()
    PATTERN_FORMATION = auto()
    FLOCKING = auto()

    @property
    def description(self) -> str:
        """Human-readable descriptions"""
        return {
            Algorithm.GATHERING: "Gathering robots to a point",
            Algorithm.SEC: "Secure Environment Coverage",
            Algorithm.PATTERN_FORMATION: "Form geometric patterns",
            Algorithm.FLOCKING: "Flocking behavior (Boids-like)"
        }[self]

class FaultType(SwarmEnum):
    """Robot fault types with visualization support"""
    CRASH = auto()
    BYZANTINE = auto()
    DELAY = auto()
    OMISSION = auto()
    SENSOR_NOISE = auto()
    
    @property
    def color(self) -> str:
        """Visualization colors for each fault type"""
        return {
            FaultType.CRASH: '#ff0000',  # Red
            FaultType.BYZANTINE: '#ffa500',  # Orange
            FaultType.DELAY: '#ffff00',  # Yellow
            FaultType.OMISSION: '#0000ff',  # Blue
            FaultType.SENSOR_NOISE: '#800080'  # Purple
        }.get(self, '#808080')  # Gray default

# --- Utility Methods ---
def get_enum_dict(enum_cls: SwarmEnum) -> Dict[str, str]:
    """Generate {name: value} dict for frontend dropdowns"""
    return {e.name: str(e) for e in enum_cls}

if __name__ == "__main__":
    # Test the enum functionality
    print("Available Algorithms:")
    for algo in Algorithm:
        print(f"- {algo}: {algo.description}")
    
    print("\nFault Colors:")
    for fault in FaultType:
        print(f"- {fault}: {fault.color}")
