# Replaces scheduler.py
from robot import (
    RobotState, Algorithm, FaultType, DistributionType, SchedulerType, # Enums
    Coordinates, Circle, SnapshotDetails, Event, Time, Id, # Typedefs
    Robot, # The Robot class itself
    SimpleLogger # If needed, or define its own
)
from typing import List, Dict, Tuple, Union, Optional
import numpy as np
import heapq
import math
import random

# Helper function (can be kept here or moved into Scheduler if preferred)
def round_coordinates(coord: Coordinates, precision: int) -> Coordinates:
    if not isinstance(coord, Coordinates):
        print(f"Warning: round_coordinates received non-Coordinates type: {type(coord)}")
        return coord
    try:
        return Coordinates(round(coord.x, precision), round(coord.y, precision))
    except Exception as e:
        print(f"Error rounding coordinates {coord}: {e}")
        return coord

class Scheduler:
    _logger = SimpleLogger() # Use logger defined in robot or define here

    def __init__(
        self,
        seed: int,
        num_of_robots: int,
        initial_positions: List[List[float]], # Keep hint using List
        robot_speeds: Union[float, List[float]], # Keep hint
        algorithm: str = Algorithm.GATHERING, # Use constant from robot
        visibility_radius: Union[float, None] = None,
        # robot_colors: Union[List[str], None] = None, # Handled by JS
        rigid_movement: bool = True,
        multiplicity_detection: bool = False,
        threshold_precision: int = 5,
        sampling_rate: float = 0.2,
        labmda_rate: float = 5,
        num_of_faults: int = 0,
        width_bound: Union[float, None] = None,
        height_bound: Union[float, None] = None,
        fault_type: str = FaultType.CRASH
    ):
        # ... (rest of __init__ remains the same, using imported types/constants)
        Scheduler._logger.info("--- Initializing Scheduler ---")
        Scheduler._logger.info(f"Seed: {seed}")
        self.seed = seed
        self.generator = np.random.default_rng(seed=seed)
        Robot._generator = self.generator # IMPORTANT: Provide generator to Robot class

        self.terminate = False
        self.rigid_movement = rigid_movement
        self.multiplicity_detection = multiplicity_detection
        self.visibility_radius = float(visibility_radius) if visibility_radius is not None else float('inf')
        self.threshold_precision = threshold_precision
        self.width_bound = float(width_bound) if width_bound else None
        self.height_bound = float(height_bound) if height_bound else None
        self.fault_type = fault_type
        self.sampling_rate = sampling_rate
        self.lambda_rate = labmda_rate
        self.robots: List[Robot] = [] # Hint with Robot from robot
        self.num_of_faults = num_of_faults

        # Process speeds
        if isinstance(robot_speeds, (float, int)):
            robot_speeds_list = [float(robot_speeds)] * num_of_robots
        elif isinstance(robot_speeds, list) and len(robot_speeds) == num_of_robots:
            robot_speeds_list = [float(s) for s in robot_speeds]
        else:
            Scheduler._logger.warning(f"Invalid robot_speeds provided ({robot_speeds}). Defaulting to 1.0.")
            robot_speeds_list = [1.0] * num_of_robots

        # Validate initial positions format
        if not isinstance(initial_positions, list) or len(initial_positions) != num_of_robots:
             raise ValueError(f"Invalid initial_positions. Expected list of {num_of_robots} coordinate pairs.")

        # Create robots
        for i in range(num_of_robots):
            try:
                 # Use Coordinates constructor from robot
                coords = Coordinates(float(initial_positions[i][0]), float(initial_positions[i][1]))
            except (IndexError, TypeError, ValueError) as e:
                 raise ValueError(f"Invalid format for initial_positions[{i}]: {initial_positions[i]}. Error: {e}")

            if algorithm == Algorithm.TWOTASK:
                # random assignment using the same generator to stay reproducible
                if self.generator.random() < 0.5:
                    task_algo = Algorithm.SEC
                    colour    = "red"
                else:
                    task_algo = Algorithm.GATHERING
                    colour    = "blue"
            else:
                task_algo = algorithm
                colour    = None
            
            # Use Robot constructor from robot
            new_robot = Robot(
                id=i,
                coordinates=coords,
                threshold_precision=threshold_precision,
                speed=robot_speeds_list[i],
                algorithm=task_algo,
                color = colour,
                visibility_radius=self.visibility_radius, # Pass float or inf
                rigid_movement=self.rigid_movement,
                multiplicity_detection=self.multiplicity_detection,
                width_bound=self.width_bound,
                height_bound=self.height_bound,
            )
            self.robots.append(new_robot)
            Scheduler._logger.info(f"Created Robot: {new_robot}")


        # Assign faults to a reproducible random subset of robots.
        if self.num_of_faults > 0 and num_of_robots > 0:
             k = min(self.num_of_faults, num_of_robots)
             faulty_indices = [int(i) for i in
                               self.generator.choice(num_of_robots, size=k, replace=False)]
             Scheduler._logger.info(f"Assigning fault '{self.fault_type}' to robots: {faulty_indices}")
             for n, idx in enumerate(faulty_indices):
                 ft = (FaultType.ALL[n % len(FaultType.ALL)]
                       if self.fault_type == "mixed" else self.fault_type)
                 self.robots[idx].set_fault(ft)
                 Scheduler._logger.info(f"  -> R{idx} fault = {ft}")
        else:
             Scheduler._logger.info("No faults requested.")


        self.visualization_snapshots: List[Tuple[Time, Dict[Id, SnapshotDetails]]] = [] # Use imported types

        # Initialize event queue (using Event from robot)
        self.priority_queue: List[Event] = []
        self.initialize_queue_exponential()

        self.current_time: Time = 0.0 # Use imported Time
        self.last_snapshot_time: Time = -1.0

        Scheduler._logger.info("--- Scheduler Initialized ---")

    # ... (rest of Scheduler methods remain the same, using imported types/constants)
    # Ensure SnapshotDetails, Event, RobotState, Coordinates etc. are used correctly
    # scheduler.py  – inside class Scheduler
    # ------------------------------------------------------------
    def _take_visualization_snapshot(self, time: Time) -> Dict[Id, SnapshotDetails]:
        """
        Capture the positions/states of all robots at the given time and
        store the snapshot in self.visualization_snapshots.
        Returns the snapshot so the caller can forward it to JS if needed.
        """
        snapshot = self.get_snapshot(time)          # current robot data
        self.visualization_snapshots.append((time, snapshot))
        return snapshot
    # ------------------------------------------------------------


    def get_snapshot(self, time: Time) -> Dict[Id, SnapshotDetails]:
        """ Creates a snapshot of the current state of all robots. """
        snapshot: Dict[Id, SnapshotDetails] = {} # Hint using imported types
        for robot in self.robots:
            current_pos = robot.get_position(time) # Returns Coordinates
            # Use SnapshotDetails constructor from robot
            snapshot[robot.id] = SnapshotDetails(
                pos=current_pos,
                state=robot.state,
                frozen=robot.frozen,
                terminated=robot.terminated,
                multiplicity=1,
                light=robot.current_light
            )

        if self.multiplicity_detection:
             self._detect_multiplicity(snapshot)

        return snapshot

    # ... other methods like _take_visualization_snapshot, generate_event, schedule_visualization_event ...
    # Ensure they use the imported Event, RobotState, Time, Id correctly.

    def generate_event(self, prev_event_time: Time, robot_id: Id, current_robot_state: str) -> None:
        # ... uses self.generator, self.lambda_rate ...
        robot = self.robots[robot_id]
        time_delta = self.generator.exponential(scale=1.0 / self.lambda_rate)
        new_activation_time = prev_event_time + max(time_delta, 1e-9)

        # Use RobotState constants from robot
        next_event_state = RobotState.LOOK
        if robot.state == RobotState.CRASH:
             next_event_state = RobotState.CRASH
        elif robot.terminated:
             Scheduler._logger.info(f"Robot R{robot_id} terminated. No new event scheduled.")
             return

        # Use Event constructor from robot
        priority_event = Event(new_activation_time, robot_id, next_event_state)
        heapq.heappush(self.priority_queue, priority_event)

    def schedule_visualization_event(self, current_time: Time):
         next_vis_time = current_time + self.sampling_rate
         # Use Event constructor from robot
         vis_event = Event(next_vis_time, -1, "VISUALIZE")
         heapq.heappush(self.priority_queue, vis_event)

    def handle_event(self) -> Tuple[int, Time, Union[Dict, None]]:
        # ... uses heapq, self.priority_queue, self.current_time ...
        # ... calls self._take_visualization_snapshot, self.schedule_visualization_event ...
        # ... uses RobotState constants ...
        # ... calls robot.look, robot.move, robot.wait, robot.set_faulty ...
        # ... calls self.generate_event, self._check_global_termination ...
        # ... returns Tuple[int, Time, Union[Dict, None]] ...

        if not self.priority_queue:
            Scheduler._logger.info("Event queue empty. Simulation likely ended.")
            self.terminate = True
            return (-1, self.current_time, None)

        current_event: Event = heapq.heappop(self.priority_queue) # Type hint Event
        time: Time = current_event.time
        robot_id: Id = current_event.id
        event_state: str = current_event.state

        if time < self.current_time:
             Scheduler._logger.warning(f"Time paradox! Event time {time:.4f} is before current time {self.current_time:.4f}. Skipping event: {current_event}")
             return (0, self.current_time, None)
        self.current_time = time

        if robot_id == -1 and event_state == "VISUALIZE":
            self._take_visualization_snapshot(time)
            self.schedule_visualization_event(time)
            latest_snapshot = self.visualization_snapshots[-1][1] if self.visualization_snapshots else None
            return (99, time, latest_snapshot)

        if robot_id < 0 or robot_id >= len(self.robots):
             Scheduler._logger.error(f"Invalid robot ID {robot_id} in event: {current_event}")
             return (0, time, None)

        robot = self.robots[robot_id]
        exit_code = 0

        # Use RobotState constants
        if robot.state == RobotState.CRASH and event_state != RobotState.CRASH:
             Scheduler._logger.info(f"T={time:.4f} R{robot_id}: Ignoring event {event_state} because robot is CRASHED.")
             self.generate_event(time, robot_id, robot.state)
             return (0, time, None)
        if robot.terminated and event_state != RobotState.TERMINATED:
             Scheduler._logger.info(f"T={time:.4f} R{robot_id}: Ignoring event {event_state} because robot is TERMINATED.")
             return (0, time, None)

        Scheduler._logger.info(f"--- T={time:.4f} Handling Event: R{robot_id} -> {event_state} ---")

        if event_state == RobotState.LOOK:
            current_snapshot = self.get_snapshot(time)
            robot.look(current_snapshot, time)

            if robot.state == RobotState.CRASH:
                 exit_code = 5
            elif robot.terminated:
                 exit_code = 4
            elif robot.frozen:
                 exit_code = 3
                 self.generate_event(time, robot_id, robot.state)
            else:
                 robot.move(time)
                 exit_code = 2
                 distance_to_target = 0.0
                 target_pos = robot.calculated_position
                 if target_pos:
                     distance_to_target = math.dist(robot.start_position, target_pos)
                 else:
                     Scheduler._logger.warning(f"T={time:.4f} R{robot_id}: Robot decided to move but has no target! Forcing WAIT.")
                     robot.wait(time)
                     exit_code = 3
                     self.generate_event(time, robot_id, robot.state)
                     return (exit_code, time, self.get_latest_snapshot())

                 move_duration = 0.0
                 if robot.speed > 1e-9:
                      # Assuming rigid movement for simplicity here based on previous version
                     move_duration = distance_to_target / robot.speed
                     if move_duration == 0 :
                          Scheduler._logger.warning(f"T={time:.4f} R{robot_id}: Zero move duration calculated but robot was not frozen. dist={distance_to_target}")

                 move_duration = max(0, move_duration)
                 wait_event_time = time + move_duration
                 if wait_event_time <= time:
                     # Sub-tick move: the distance is so small (or t so large) that
                     # the arrival time does not advance the clock. Do NOT schedule a
                     # zero-duration MOVE -- it would strand the robot and livelock
                     # the simulation. Finalise the move now so it reaches its target.
                     robot.wait(time)
                     exit_code = 3
                     self.generate_event(time, robot_id, robot.state)
                     Scheduler._logger.info(f"T={time:.4f} R{robot_id}: Sub-tick move finalised immediately (dist={distance_to_target:.3e}).")
                 else:
                     # Use Event constructor
                     wait_event = Event(wait_event_time, robot_id, RobotState.WAIT)
                     heapq.heappush(self.priority_queue, wait_event)
                     Scheduler._logger.info(f"T={time:.4f} R{robot_id}: Scheduled WAIT event at T={wait_event_time:.4f} (duration {move_duration:.4f})")

        elif event_state == RobotState.WAIT:
             robot.wait(time)
             exit_code = 3
             self.generate_event(time, robot_id, robot.state)

        elif event_state == RobotState.MOVE:
             Scheduler._logger.warning(f"T={time:.4f} R{robot_id}: Received unexpected MOVE event. State: {robot.state}. Ignoring.")
             exit_code = 0

        elif event_state == RobotState.CRASH:
             robot.set_faulty(True)
             exit_code = 5
             Scheduler._logger.info(f"T={time:.4f} R{robot_id}: Marked as CRASHED by event.")

        else:
             Scheduler._logger.warning(f"T={time:.4f} R{robot_id}: Unhandled event state '{event_state}'")
             exit_code = 0

        if self._check_global_termination():
             Scheduler._logger.info(f"--- T={time:.4f} Global Termination Condition Met ---")
             self.terminate = True
             return (-1, time, self.get_snapshot(time))

        latest_snapshot = self.visualization_snapshots[-1][1] if self.visualization_snapshots else self.get_snapshot(time)
        return (exit_code, time, latest_snapshot)


    def initialize_queue_exponential(self) -> None:
        # ... uses self.generator, self.lambda_rate, self.robots ...
        # ... uses RobotState, Event, Time, Id ...
        Scheduler._logger.info("Initializing event queue with exponential distribution...")
        initial_times = self.generator.exponential(scale=1.0 / self.lambda_rate, size=len(self.robots))
        self.priority_queue = []
        for i, robot in enumerate(self.robots):
             initial_state = RobotState.LOOK
             if robot.state == RobotState.CRASH:
                  initial_state = RobotState.CRASH
             event_time = max(0, initial_times[i])
             # Use Event constructor
             event = Event(event_time, robot.id, initial_state)
             self.priority_queue.append(event)
             Scheduler._logger.info(f"  Initial event for R{robot.id}: {initial_state} at T={event_time:.4f}")

        self.schedule_visualization_event(0.0)
        heapq.heapify(self.priority_queue)
        Scheduler._logger.info("Event queue initialized.")


    def _check_global_termination(self) -> bool:
        # ... uses self.robots, RobotState ...
        num_non_crashed = 0
        all_terminated = True
        for robot in self.robots:
            # Byzantine robots never settle; correct robots must gather despite them.
            if robot.state != RobotState.CRASH and robot.fault_type != FaultType.BYZANTINE:
                num_non_crashed += 1
                if not robot.terminated:
                    all_terminated = False
                    break # No need to check further

        if num_non_crashed == 0 and len(self.robots) > 0:
             Scheduler._logger.info("Global termination: All robots have crashed.")
             return True
        if num_non_crashed > 0 and all_terminated:
             Scheduler._logger.info("Global termination: All non-crashed robots are terminated.")
             return True

        return False

    def _detect_multiplicity(self, snapshot: Dict[Id, SnapshotDetails]):
        # ... uses SnapshotDetails, Coordinates, Id, math.dist, round_coordinates ...
        positions_list: List[Tuple[Coordinates, Id]] = []
        for robot_id, details in snapshot.items():
             if details.pos:
                positions_list.append((details.pos, robot_id))

        positions_list.sort(key=lambda item: (item[0].x, item[0].y)) # Access .x, .y

        n = len(positions_list)
        visited = [False] * n
        multiplicity_groups: List[List[Id]] = []

        for i in range(n):
            if visited[i]: continue
            current_group = [positions_list[i][1]]
            visited[i] = True
            # pos1_rounded = round_coordinates(positions_list[i][0], self.threshold_precision) # Can use rounding or dist

            for j in range(i + 1, n):
                if visited[j]: continue
                # pos2_rounded = round_coordinates(positions_list[j][0], self.threshold_precision)
                # if pos1_rounded == pos2_rounded:

                # Use distance check which might be more robust
                distance = math.dist(positions_list[i][0], positions_list[j][0])
                if distance < math.pow(10, -self.threshold_precision):
                    visited[j] = True
                    current_group.append(positions_list[j][1])

            if len(current_group) > 0:
                multiplicity_groups.append(current_group)

        for group in multiplicity_groups:
            count = len(group)
            if count > 1:
                for robot_id in group:
                    details = snapshot[robot_id]
                    # Use _replace for NamedTuple update
                    snapshot[robot_id] = details._replace(multiplicity=count)


    def get_latest_snapshot(self) -> Union[Dict[Id, SnapshotDetails], None]:
        if self.visualization_snapshots:
            return self.visualization_snapshots[-1][1]
        else:
            return self.get_snapshot(self.current_time)


    def get_all_robot_data_for_js(self) -> List[dict]:
        robot_data = []
        latest_snap = self.get_latest_snapshot()

        for robot in self.robots:
            pos = robot.get_position(self.current_time)
            multiplicity = 1
            if latest_snap and robot.id in latest_snap:
                multiplicity = latest_snap[robot.id].multiplicity or 1

            sec_data = None
            if robot.sec:
                sec_data = {"center_x": robot.sec.center.x,
                            "center_y": robot.sec.center.y,
                            "radius":   robot.sec.radius}

            target_x = robot.calculated_position.x if robot.calculated_position else None
            target_y = robot.calculated_position.y if robot.calculated_position else None

            robot_data.append({
                "id": robot.id,
                "x": pos.x,
                "y": pos.y,
                "state": robot.state,
                "frozen": robot.frozen,
                "terminated": robot.terminated,
                "crashed": robot.state == RobotState.CRASH,
                "multiplicity": multiplicity,
                "sec": sec_data,
                "target_x": target_x,
                "target_y": target_y,
                "speed": robot.speed,
                "color": robot.color,                       # << NEW
                "fault_type": robot.fault_type,
                "light": robot.current_light,
                "visibility_radius": (robot.visibility_radius
                                      if robot.visibility_radius != float("inf") else None)
            })
        return robot_data
