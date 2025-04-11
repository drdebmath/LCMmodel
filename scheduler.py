from enums import *
from type_defs import *
from robot import Robot
import numpy as np
import heapq
import math
import logging
import random


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
        robot_faults: list[dict] = None,
    ):
        Scheduler._logger = logger
        self.seed = seed
        self.random_gen = random.Random(seed)
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
        self.lambda_rate = labmda_rate
        self.robots: list[Robot] = []
        self.fault_status = {}  # Track fault state for each robot

        # Initialize robots
        if isinstance(robot_speeds, float) or isinstance(robot_speeds, int):
            robot_speeds_list = [robot_speeds] * num_of_robots
        else:
            robot_speeds_list = robot_speeds

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
            self.robots.append(new_robot)
            
            # Initialize fault status
            self.fault_status[i] = {
                'active': False,
                'triggered': False,
                'delay_counter': 0
            }

        # Apply faults if specified
        if robot_faults:
            for i, fault_info in enumerate(robot_faults):
                if i < len(self.robots):
                    self.robots[i].fault_type = fault_info.get('type', 'None')
                    self.robots[i].fault_probability = fault_info.get('probability', 0) / 100.0

        self.initialize_queue_exponential()
        Robot._generator = self.generator

    def _should_trigger_fault(self, robot_id: int) -> bool:
        """Check if fault should be triggered based on probability"""
        robot = self.robots[robot_id]
        if (robot.fault_type != 'None' and 
            not self.fault_status[robot_id]['triggered']):
            return self.random_gen.random() < robot.fault_probability
        return False

    def _apply_fault_effects(self, robot: Robot, event_state: RobotState) -> bool:
        """Apply fault effects. Returns True if event should be skipped"""
        if robot.fault_type == 'None':
            return False
            
        fid = robot.id
        fault = self.fault_status[fid]
        
        # Check if fault should trigger
        if not fault['triggered'] and self._should_trigger_fault(fid):
            fault['triggered'] = True
            fault['active'] = True
            robot.fault_status = 'Triggered'
            self._logger.info(f"Fault triggered for robot {fid}: {robot.fault_type}")
        
        # Apply active fault effects
        if fault['active']:
            robot.fault_status = 'Active'
            
            if robot.fault_type == 'Crash':
                robot.frozen = True
                robot.terminated = True
                return True
                
            elif robot.fault_type == 'Delay':
                fault['delay_counter'] += 1
                if fault['delay_counter'] % 2 == 1:  # Skip every other move
                    return True
                    
            elif robot.fault_type == 'Byzantine' and event_state == RobotState.MOVE:
                # Random direction movement
                angle = self.random_gen.uniform(0, 2 * math.pi)
                distance = robot.speed * self.sampling_rate
                robot.calculated_position = Coordinates(
                    robot.start_position.x + distance * math.cos(angle),
                    robot.start_position.y + distance * math.sin(angle)
                )
                
        return False

    def handle_event(self) -> int:
        exit_code = -1

        if len(self.priority_queue) == 0:
            return exit_code

        current_event = heapq.heappop(self.priority_queue)
        event_state = current_event.state
        time = current_event.time

        if event_state is None:
            self.get_snapshot(time, visualization_snapshot=True)
            exit_code = 0
        else:
            robot = self.robots[current_event.id]
            
            # Apply fault effects before processing event
            if self._apply_fault_effects(robot, event_state):
                self.generate_event(current_event)
                return exit_code
                
            if event_state == RobotState.LOOK:
                robot.state = RobotState.LOOK
                robot.look(self.get_snapshot(time), time)
                if robot.terminated:
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

    def get_snapshot(
        self, time: float, visualization_snapshot: bool = False
    ) -> dict[int, SnapshotDetails]:
        snapshot = {}
        for robot in self.robots:
            visible_robots = self._get_visible_robots(robot, time)
            snapshot[robot.id] = SnapshotDetails(
                robot.get_position(time),
                robot.state,
                robot.frozen,
                robot.terminated,
                1,
                robot.fault_type,
                getattr(robot, 'fault_status', 'None')
            )

        self._detect_multiplicity(snapshot)
        if visualization_snapshot:
            self.visualization_snapshots.append((time, snapshot))
        else:
            self.snapshot_history.append((time, snapshot))

        return snapshot

    def _get_visible_robots(self, observer: Robot, time: float) -> dict[int, SnapshotDetails]:
        visible = {}
        for robot in self.robots:
            if robot.id == observer.id:
                continue
                
            if self._is_visible(observer, robot, time):
                visible[robot.id] = SnapshotDetails(
                    robot.get_position(time),
                    robot.state,
                    robot.frozen,
                    robot.terminated,
                    1,
                    robot.fault_type,
                    getattr(robot, 'fault_status', 'None')
                )
        return visible

    def _is_visible(self, observer: Robot, target: Robot, time: float) -> bool:
        observer_pos = observer.get_position(time)
        target_pos = target.get_position(time)
        distance = math.dist(observer_pos, target_pos)

        if observer.visibility_radius and distance > observer.visibility_radius:
            return False
            
        if observer.obstructed_visibility:
            for other_robot in self.robots:
                if other_robot.id in [observer.id, target.id]:
                    continue
                    
                other_pos = other_robot.get_position(time)
                if self._is_between(observer_pos, target_pos, other_pos):
                    return False
        return True

    def _is_between(
        self, 
        a: Coordinates, 
        b: Coordinates, 
        c: Coordinates,
        threshold: float = 0.1
    ) -> bool:
        ac = math.dist(a, c)
        bc = math.dist(b, c)
        ab = math.dist(a, b)
        return abs(ac + bc - ab) < threshold

    def generate_event(self, current_event: Event) -> None:
        if current_event.state is None and len(self.priority_queue) > 0:
            new_event_time = current_event.time + self.sampling_rate
            new_event = Event(new_event_time, -1, None)
            heapq.heappush(self.priority_queue, new_event)
            return

        new_event_time = 0.0
        robot = self.robots[current_event.id]

        if current_event.state == RobotState.MOVE:
            distance = 0.0
            if self.rigid_movement:
                distance = math.dist(robot.calculated_position, robot.start_position)
            else:
                percentage = 1 - self.generator.uniform()
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

    def initialize_queue(self) -> None:
        lambda_value = 5
        generator = np.random.default_rng()
        num_samples = 2
        poisson_numbers = generator.poisson(lambda_value, num_samples)
        Scheduler._logger.info(poisson_numbers)

    def initialize_queue_exponential(self) -> None:
        Scheduler._logger.info(f"Seed used: {self.seed}")
        self.generator = np.random.default_rng(seed=self.seed)
        num_of_events = len(self.robots)
        time_intervals = self.generator.exponential(
            scale=1 / self.lambda_rate, size=num_of_events
        )
        Scheduler._logger.info(f"Time intervals between events: {time_intervals}")

        initial_event = Event(0.0, -1, None)
        self.priority_queue: list[Event] = [initial_event]

        for robot in self.robots:
            time = time_intervals[robot.id]
            event = Event(time, robot.id, robot.state.next_state())
            self.priority_queue.append(event)

        heapq.heapify(self.priority_queue)

    def _all_robots_reached(self) -> bool:
        for robot in self.robots:
            if not robot.frozen:
                return False
        return True

    def _detect_multiplicity(self, snapshot: dict[int, SnapshotDetails]):
        positions = [(v.pos, k) for k, v in snapshot.items()]
        positions.sort()

        i = 0
        multiplicity = 1
        while i < len(positions):
            multiplicity_group = [positions[i][1]]
            rounded_coordinates1 = round_coordinates(
                positions[i][0], self.threshold_precision - 2
            )

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

            for robot_id in multiplicity_group:
                snapshot_details = list(snapshot[robot_id])
                snapshot_details[4] = multiplicity
                snapshot[robot_id] = SnapshotDetails(*snapshot_details)

            i += len(multiplicity_group)
            multiplicity = 1


def round_coordinates(coord: Coordinates, precision: int):
    return Coordinates(round(coord.x, precision), round(coord.y, precision))
