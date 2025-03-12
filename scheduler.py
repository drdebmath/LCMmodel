from enums import *
from type_defs import *
from robot import Robot
import numpy as np
import heapq
import math
import logging


class Scheduler:

    _logger: logging.Logger | None = None

    def __init__(
        self,
        logger: logging.Logger,
        seed: int,
        num_of_robots: int,
        initial_positions: list[list[float]] | None,
        robot_speeds: float | list[float],
        algorithm: str = Algorithm.GATHERING,
        visibility_radius: float | list[float] | None = None,
        robot_orientations: list[Orientation] | None = None,
        robot_colors: list[str] | None = None,
        obstructed_visibility: bool = False,
        rigid_movement: bool = True,
        multiplicity_detection: bool = False,
        probability_distribution: str = DistributionType.EXPONENTIAL,
        scheduler_type: str = SchedulerType.ASYNC,
        threshold_precision: int = 5,
        sampling_rate: float = 0.2,
        labmda_rate: float = 5,
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
        self.snapshot_history: list[tuple[Time, dict[int, SnapshotDetails]]] = []
        self.visualization_snapshots: list[tuple[Time, dict[int, SnapshotDetails]]] = []
        self.sampling_rate = sampling_rate
        self.lambda_rate = labmda_rate  # Average number of events per time unit
        self.robots: list[Robot] = []

        if isinstance(robot_speeds, float) or isinstance(robot_speeds, int):
            robot_speeds_list = [robot_speeds] * num_of_robots

        for i in range(num_of_robots):
            new_robot = Robot(
                logger=logger,
                id=i,
                coordinates=Coordinates(*initial_positions[i]),
                threshold_precision=threshold_precision,
                speed=robot_speeds_list[i],
                algorithm=algorithm,
                visibility_radius=self.visibility_radius,
                rigid_movement=self.rigid_movement,
            )
            new_robot.fault_type = self._assign_fault_type(i)  # Assign fault dynamically
            self.robots.append(new_robot)
            
self.initialize_queue_exponential()
Robot._generator = self.generator 
				
def _assign_fault_type(self, robot_id: int) -> str | None:
    """Assigns fault properties to robots based on probability."""
    fault_types = ["crash", "byzantine", "delay", None]  # Possible faults
    probabilities = [0.2, 0.2, 0.2, 0.4]  # Adjust probabilities as needed

    assigned_fault = np.random.choice(fault_types, p=probabilities)
    Scheduler._logger.info(f"Assigning fault {assigned_fault} to Robot {robot_id}")
    return assigned_fault

    def get_snapshot(
        self, time: float, visualization_snapshot: bool = False
    ) -> dict[int, SnapshotDetails]:
        snapshot = {}
        for robot in self.robots:
            snapshot[robot.id] = SnapshotDetails(
                robot.get_position(time), robot.state, robot.frozen, robot.terminated, 1
            )

        self._detect_multiplicity(snapshot)  # in-place
        if visualization_snapshot:
            self.visualization_snapshots.append((time, snapshot))
        else:
            self.snapshot_history.append((time, snapshot))

        return snapshot

    def generate_event(self, current_event: Event) -> None:
        # Visualization events
        if current_event.state == None and len(self.priority_queue) > 0:
            new_event_time = current_event.time + self.sampling_rate
            new_event = Event(new_event_time, -1, None)
            heapq.heappush(self.priority_queue, new_event)
            return

        new_event_time = 0.0
        robot = self.robots[current_event.id]

        # Robot will definitely reach calculated position
        if current_event.state == RobotState.MOVE:
            distance = 0.0
            if self.rigid_movement == True:
                distance = math.dist(robot.calculated_position, robot.start_position)
            else:
                percentage = 1 - self.generator.uniform()  # range of values is (0,1]
                Scheduler._logger.info(f"percentage of journey: {percentage}")
                distance = percentage * math.dist(
                    robot.calculated_position, robot.start_position
                )
            new_event_time = current_event.time + (distance / robot.speed)
        else:
            new_event_time = current_event.time + self.generator.exponential(
                scale=1 / self.lambda_rate
            )

        new_event_state = robot.state.next_state()

        priority_event = Event(new_event_time, current_event.id, new_event_state)

        heapq.heappush(self.priority_queue, priority_event)

    def handle_event(self) -> int:
        exit_code = -1

        if len(self.priority_queue) == 0:
            return exit_code

        current_event = heapq.heappop(self.priority_queue)

        event_state = current_event.state

        time = current_event.time

        if event_state == None:
            self.get_snapshot(time, visualization_snapshot=True)
            exit_code = 0
        else:
            robot = self.robots[current_event.id]
            
            # Crash Fault: Skip execution
if robot.fault_type == "crash":
    Scheduler._logger.info(f"R{robot.id} is crashed and will not act this round.")
    return exit_code  # Skip execution for crashed robots
	
# Byzantine Fault: Modify snapshot before execution
if robot.fault_type == "byzantine":
    robot.snapshot = self._introduce_byzantine_error(robot.snapshot)
    Scheduler._logger.info(f"R{robot.id} is Byzantine and is sending false data.")

# Delayed Response Fault: 30% chance to skip movement
if robot.fault_type == "delay" and np.random.random() < 0.3:
    Scheduler._logger.info(f"R{robot.id} is delayed and skipping movement this round.")
		return exit_code  # Skip movement step

if event_state == RobotState.LOOK:
                robot.state = RobotState.LOOK
                robot.look(self.get_snapshot(time), time)

                # Removes robot from simulation
                if robot.terminated == True:
                    return 4
                exit_code = 1
elif event_state == RobotState.MOVE:
                robot.move(time)
                exit_code = 2
elif event_state == RobotState.WAIT:
                robot.wait(time)
                exit_code = 3

self.generate_event(current_event)
return exit_code

def initialize_queue(self) -> None:
        # Set the lambda parameter (average rate of occurrences)
        lambda_value = 5  # 5 occurrences per interval

        # Generate Poisson-distributed random numbers
        generator = np.random.default_rng()
        num_samples = 2  # Total number of samples to generate
        poisson_numbers = generator.poisson(lambda_value, num_samples)

        # Display the generated numbers
        Scheduler._logger.info(poisson_numbers)

def initialize_queue_exponential(self) -> None:
        Scheduler._logger.info(f"Seed used: {self.seed}")

        # Generate time intervals for n events
        self.generator = np.random.default_rng(seed=self.seed)
        num_of_events = len(self.robots)
        time_intervals = self.generator.exponential(
            scale=1 / self.lambda_rate, size=num_of_events
        )
        Scheduler._logger.info(f"Time intervals between events: {time_intervals}")

        initial_event = Event(0.0, -1, None)  # initial event for visualization
        self.priority_queue: list[Event] = [initial_event]

        for robot in self.robots:
            time = time_intervals[robot.id]
            event = Event(time, robot.id, robot.state.next_state())
            self.priority_queue.append(event)

        heapq.heapify(self.priority_queue)

def _all_robots_reached(self) -> bool:
        for robot in self.robots:
            if robot.frozen == False:
                return False
        return True

    # Can be improved when it comes to precision/detection
def _detect_multiplicity(self, snapshot: dict[int, SnapshotDetails]):
        positions = [(v.pos, k) for k, v in snapshot.items()]

        positions.sort()

        i = 0
        multiplicity = 1
        while i < len(positions):
            multiplicity_group = [positions[i][1]]  # Start a new group
            rounded_coordinates1 = round_coordinates(
                positions[i][0], self.threshold_precision - 2
            )

            # Check for close positions
            for j in range(i + 1, len(positions)):
                rounded_coordinates2 = round_coordinates(
                    positions[j][0], self.threshold_precision - 2
                )

                is_close = all(
                    abs(rounded_coordinates1[x] - rounded_coordinates2[x])
                    <= 10**-self.threshold_precision
                    for x in range(2)
                )

                if is_close:
                    multiplicity += 1
                    multiplicity_group.append(positions[j][1])
                else:
                    break

            # Update multiplicity for all robots in the group
            for robot_id in multiplicity_group:
                snapshot_details = list(snapshot[robot_id])
                snapshot_details[4] = multiplicity
                snapshot[robot_id] = SnapshotDetails(*snapshot_details)

            # Move to the next group
            i += len(multiplicity_group)
            multiplicity = 1


def round_coordinates(coord: Coordinates, precision: int):

    return Coordinates(round(coord.x, precision), round(coord.y, precision))
def _introduce_byzantine_error(self, snapshot):
    """Byzantine robots modify snapshot data to mislead others."""
    for key in snapshot.keys():
        if np.random.random() < 0.5:  # 50% chance to corrupt data
            snapshot[key].pos = Coordinates(np.random.uniform(-100, 100), np.random.uniform(-100, 100))
    return snapshot
