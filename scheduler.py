from enums import Algorithm, DistributionType, SchedulerType, FaultType, RobotState
from type_defs import Coordinates, Orientation, SnapshotDetails
from robot import Robot, PositionGrid
from typing import Optional, Union, Dict, List, Set
import numpy as np
import heapq
import math
import logging
from collections import defaultdict
import random


class Scheduler:
    _logger: Optional[logging.Logger] = None
    _position_grid: Optional[PositionGrid] = None  # Spatial grid for visibility optimization

    def __init__(
        self,
        logger: logging.Logger,
        seed: int,
        num_of_robots: int,
        initial_positions: List[List[float]],
        robot_speeds: Union[float, List[float]],
        algorithm: Algorithm = Algorithm.GATHERING,  # Changed to use Enum directly
        visibility_radius: Union[float, List[float]] = 10.0,
        robot_orientations: Optional[List[Orientation]] = None,
        robot_colors: Optional[List[str]] = None,
        obstructed_visibility: bool = False,
        rigid_movement: bool = True,
        multiplicity_detection: bool = False,
        probability_distribution: DistributionType = DistributionType.EXPONENTIAL,  # Enum
        scheduler_type: SchedulerType = SchedulerType.ASYNC,  # Enum
        threshold_precision: int = 5,
        sampling_rate: float = 0.2,
        labmda_rate: float = 5,
        fault_prob: float = 0.3  # Probability of a robot having a fault
    ):
        Scheduler._logger = logger
        self.seed = seed
        self.terminate = False
        self.rigid_movement = rigid_movement
        self.multiplicity_detection = multiplicity_detection
        self.probability_distribution = probability_distribution
        self.scheduler_type = scheduler_type
        self.robot_speeds = robot_speeds
        self.visibility_radius = visibility_radius
        self.robot_orientations = robot_orientations
        self.robot_colors = robot_colors
        self.obstructed_visibility = obstructed_visibility
        self.threshold_precision = threshold_precision
        self.snapshot_history: List[Tuple[float, Dict[int, SnapshotDetails]]] = []
        self.visualization_snapshots: List[Tuple[float, Dict[int, SnapshotDetails]]] = []
        self.sampling_rate = sampling_rate
        self.lambda_rate = labmda_rate
        self.fault_prob = max(0.0, min(fault_prob, 1.0))  # Clamped between 0-1
        self.robots: List[Robot] = []
        self.priority_queue = []  # Added explicit declaration

        # Initialize spatial grid (2x visibility radius for optimal performance)
        if visibility_radius:
            cell_size = (visibility_radius * 2) if isinstance(visibility_radius, float) else (max(visibility_radius) * 2)
            Scheduler._position_grid = PositionGrid(cell_size=cell_size)

        # Initialize robot speeds
        robot_speeds_list = [robot_speeds] * num_of_robots if isinstance(robot_speeds, (float, int)) else robot_speeds

        # Initialize robots with potential faults
        for i in range(num_of_robots):
            fault = self._assign_fault_type(i) if random.random() < self.fault_prob else None
            new_robot = Robot(
                logger=logger,
                id=i,
                coordinates=Coordinates(*initial_positions[i]),
                algorithm=algorithm,
                speed=robot_speeds_list[i],
                visibility_radius=visibility_radius[i] if isinstance(visibility_radius, list) else visibility_radius,
                obstructed_visibility=obstructed_visibility,
                rigid_movement=rigid_movement,
                threshold_precision=threshold_precision,
                fault_type=fault  # Using enum type directly
            )
            self.robots.append(new_robot)

        self.initialize_queue_exponential()
        Robot._generator = np.random.default_rng(seed=self.seed)

    def get_snapshot(
        self, 
        time: float, 
        visualization_snapshot: bool = False
    ) -> Dict[int, SnapshotDetails]:
        """Get current state of all robots, with spatial optimization"""
        snapshot = {}
        
        # Update all positions in spatial grid first
        if Scheduler._position_grid:
            for robot in self.robots:
                Scheduler._position_grid.update_position(robot.id, robot.get_position(time))

        for robot in self.robots:
            snapshot[robot.id] = SnapshotDetails(
                robot.get_position(time),
                robot.state,
                robot.frozen,
                robot.terminated,
                1,  # multiplicity
                robot.fault_type  # Added fault type to snapshot
            )

        self._detect_multiplicity(snapshot)
        
        if visualization_snapshot:
            self.visualization_snapshots.append((time, snapshot))
        else:
            self.snapshot_history.append((time, snapshot))

        return snapshot

    def _is_visible(self, observer: Robot, target: Robot) -> bool:
        """Optimized visibility check with spatial grid"""
        if observer.visibility_radius is None:
            return True

        observer_pos = observer.coordinates
        target_pos = target.coordinates
        
        # Fast distance check first
        if math.dist((observer_pos.x, observer_pos.y), (target_pos.x, target_pos.y)) > observer.visibility_radius:
            return False

        # Full obstruction check if needed
        if self.obstructed_visibility:
            if Scheduler._position_grid:
                nearby_ids = Scheduler._position_grid.get_nearby(
                    observer_pos, 
                    observer.visibility_radius
                )
                for robot_id in nearby_ids:
                    if robot_id == observer.id or robot_id == target.id:
                        continue
                    obst = self.robots[robot_id]
                    if self._is_between(observer_pos, obst.coordinates, target_pos):
                        return False
            else:
                for obst in self.robots:
                    if obst.id == observer.id or obst.id == target.id:
                        continue
                    if self._is_between(observer_pos, obst.coordinates, target_pos):
                        return False
        return True

    def _is_between(self, a: Coordinates, b: Coordinates, c: Coordinates) -> bool:
        """Check if point b is between a and c (collinear and within segment)"""
        cross = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y)
        if abs(cross) > 1e-6:
            return False
            
        dot = (b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)
        if dot < 0:
            return False
            
        squared_len = (c.x - a.x)**2 + (c.y - a.y)**2
        return dot <= squared_len

    def handle_event(self) -> int:
        """Process next event with enhanced fault handling"""
        if not self.priority_queue:
            return -1

        current_event = heapq.heappop(self.priority_queue)
        time = current_event.time

        if current_event.state is None:
            self.get_snapshot(time, visualization_snapshot=True)
            return 0

        robot = self.robots[current_event.id]

        # Enhanced fault handling with enum checks
        if robot.fault_type == FaultType.CRASH:
            if current_event.state == RobotState.MOVE:
                robot.frozen = True
                Scheduler._logger.info(f"[{time}] R{robot.id} CRASHED")
            return -1
        elif robot.fault_type == FaultType.DELAY and current_event.state == RobotState.MOVE:
            if random.random() < 0.3:  # 30% chance to delay
                Scheduler._logger.info(f"[{time}] R{robot.id} DELAYED")
                return -1

        # Normal event processing
        if current_event.state == RobotState.LOOK:
            robot.state = RobotState.LOOK
            snapshot = self.get_snapshot(time)
            
            if robot.fault_type == FaultType.BYZANTINE:
                snapshot = robot._introduce_byzantine_error(snapshot)
                
            robot.look(snapshot, time)
            return 1 if not robot.terminated else 4
        elif current_event.state == RobotState.MOVE:
            robot.move(time)
            return 2
        elif current_event.state == RobotState.WAIT:
            robot.wait(time)
            return 3

        self.generate_event(current_event)
        return -1

    def initialize_queue_exponential(self) -> None:
        """Initialize event queue with exponential distribution"""
        self.generator = np.random.default_rng(self.seed)
        self.priority_queue = []
        
        for robot in self.robots:
            event_time = self.generator.exponential(scale=1.0/self.lambda_rate)
            heapq.heappush(self.priority_queue, (event_time, robot.id, RobotState.LOOK))

    def generate_event(self, event: Tuple[float, int, RobotState]) -> None:
        """Generate next event for a robot based on current state"""
        time, robot_id, state = event
        robot = self.robots[robot_id]
        
        if state == RobotState.LOOK:
            next_time = time + self.generator.exponential(scale=1.0/self.lambda_rate)
            heapq.heappush(self.priority_queue, (next_time, robot_id, RobotState.MOVE))
        elif state == RobotState.MOVE:
            next_time = time + (robot.travelled_distance / robot.speed)
            heapq.heappush(self.priority_queue, (next_time, robot_id, RobotState.WAIT))

    def _detect_multiplicity(self, snapshot: Dict[int, SnapshotDetails]) -> None:
        """Detect if multiple robots occupy the same position"""
        if not self.multiplicity_detection:
            return
            
        pos_counts = defaultdict(int)
        for details in snapshot.values():
            pos_counts[(details.pos.x, details.pos.y)] += 1
            
        for rid, details in snapshot.items():
            if pos_counts[(details.pos.x, details.pos.y)] > 1:
                snapshot[rid] = details._replace(multiplicity=pos_counts[(details.pos.x, details.pos.y)])

    def _assign_fault_type(self, robot_id: int) -> Optional[FaultType]:
        """Assign random fault based on probability using Enum"""
        if random.random() < self.fault_prob:
            return random.choice(list(FaultType))
        return None


class PositionGrid:
    """Optimized spatial partitioning for visibility checks"""
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.grid = defaultdict(set)
        
    def update_position(self, robot_id: int, pos: Coordinates) -> None:
        cell = (int(pos.x // self.cell_size), int(pos.y // self.cell_size))
        self.grid[cell].add(robot_id)
        
    def get_nearby(self, pos: Coordinates, radius: float) -> Set[int]:
        nearby = set()
        min_x = int((pos.x - radius) // self.cell_size)
        max_x = int((pos.x + radius) // self.cell_size)
        min_y = int((pos.y - radius) // self.cell_size)
        max_y = int((pos.y + radius) // self.cell_size)
        
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                nearby.update(self.grid.get((x, y), set()))
        return nearby


def round_coordinates(coord: Coordinates, precision: int) -> Coordinates:
    return Coordinates(round(coord.x, precision), round(coord.y, precision))
