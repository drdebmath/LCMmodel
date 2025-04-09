from enums import RobotState, Algorithm
from type_defs import *
from typing import Callable
import math
import logging
import time
import random

import algorithms  # for GATHERING algorithm

class Robot:
    _logger: logging.Logger | None = None
    _generator = None

    # A tiny distance below which we consider the robot "not moving further."
    _EPSILON_DISTANCE = 1e-5
    
    # Cache for expensive calculations
    _calculation_cache = {}

    # Color mapping from hex to named colors
    _HEX_TO_COLOR = {
        "#000000": "black",
        "#FFFFFF": "white",
        "#FF0000": "red",
        "#0000FF": "blue",
        "#00FF00": "green",
        "#FFFF00": "yellow",
        "#FF00FF": "magenta",
        "#00FFFF": "cyan"
    }

    def __init__(
        self,
        logger: logging.Logger,
        id: int,
        coordinates: Coordinates,
        algorithm: str,
        custom_alg: str,
        custom_term_code: str,
        speed: float = 1.0,
        color: str | None = None,
        visibility_radius: float | None = None,
        orientation: Orientation | None = None,
        obstructed_visibility: bool = False,
        multiplicity_detection: bool = False,
        rigid_movement: bool = False,
        threshold_precision: float = 5,
        num_of_robots: int = 2,
    ):
        Robot._logger = logger
        self.id = id
        self.coordinates = coordinates
        self.algorithm_str = algorithm
        self.custom_alg = custom_alg
        self.custom_term_code = custom_term_code
        self.speed = speed
        self.innate_color = color  # Store the robot's innate color
        self.visibility_radius = visibility_radius
        self.obstructed_visibility = obstructed_visibility
        self.multiplicity_detection = multiplicity_detection
        self.rigid_movement = rigid_movement
        self.orientation = orientation
        self.threshold_precision = threshold_precision
        self.num_of_robots = num_of_robots  # Store the number of robots

        self.start_time = None
        self.end_time = None
        self.state = RobotState.WAIT
        self.start_position = coordinates
        self.calculated_position = coordinates
        self.snapshot: dict[Id, SnapshotDetails] | None = None
        self.travelled_distance = 0.0
        self.frozen = False
        self.terminated = False
        self.sec = None  # For SEC algorithm
        self.epoch = 0   # Track epoch for this robot
        
        # Light Attributes - CRITICAL: Ensure initial color is set correctly
        self.current_light = color  # Start with the provided color
        self.last_light_event_time = 0.0
        self.light_changes_count = 0  # Track number of light changes
        self.true_final_color = color  # Initialize true final color

        # Cached visibility info - robots visible to this robot
        self.visible_robots_cache = set()
        self.last_visibility_update_time = -1.0
        
        # Position interpolation cache
        self.interpolation_cache = {}
        
        # Movement tracking
        self.last_significant_movement_time = 0.0
        self.movement_since_last_check = 0.0

        # For tracking color change delays in TwoRobot algorithm
        self.color_change_timer = 0.0
        self.last_color_check_time = 0.0

        # Convert string to enum
        match algorithm:
            case "Gathering":
                self.algorithm = Algorithm.GATHERING
            case "SEC":
                self.algorithm = Algorithm.SEC
            case "Custom":
                self.algorithm = Algorithm.CUSTOM
            case "ColorBased":
                self.algorithm = Algorithm.COLOR_BASED
                # For ColorBased, make sure the current_light is the innate_color
                self.current_light = self.innate_color
            case "TwoRobot":
                self.algorithm = Algorithm.TWO_ROBOT
                # For TwoRobot, initialize with the specified color
                self.current_light = self.innate_color
                Robot._logger.info(f"TwoRobot robot {id} initialized with color {self.innate_color} (current_light={self.current_light})")
            case "MutualVisibility":
                self.algorithm = Algorithm.MUTUAL_VISIBILITY
                # For MutualVisibility, initialize with red color
                self.current_light = "red"
                self.innate_color = "red"
                Robot._logger.info(f"MutualVisibility robot {id} initialized with color red")

    def set_light(self, new_color: str, simulation_time: float) -> None:
        """
        Change the robot's light color and record light change events.
        For TwoRobot algorithm:
        - Both robots start as black
        - When a robot changes color, update both current_light and true_final_color
        - Ensure light changes are counted for statistics
        - Never change to gray for TwoRobot algorithm
        
        For MutualVisibility algorithm:
        - Robots can be red (obstructing), blue (sees collinear), or green (clear visibility)
        
        Args:
            new_color: The new color to set
            simulation_time: Current simulation time
        """
        # CRITICAL: Never change to gray for TwoRobot or MutualVisibility algorithms
        if self.algorithm in [Algorithm.TWO_ROBOT, Algorithm.MUTUAL_VISIBILITY]:
            if new_color == "gray":
                Robot._logger.info(f"[{simulation_time}] R{self.id} {self.algorithm.name} algorithm - ignoring gray color change")
                return
            
        # For TwoRobot algorithm, only allow changes to white
        if self.algorithm == Algorithm.TWO_ROBOT:
            if new_color != "white" and self.current_light != "white":
                Robot._logger.info(f"[{simulation_time}] R{self.id} TwoRobot algorithm - ignoring non-white color change")
                return
            
        # Track the light change for statistics purposes
        if new_color != self.current_light:
            # Always count the light change for statistics
            self.light_changes_count += 1
            self.last_light_event_time = simulation_time
            
            # Log the light change event
            Robot._logger.info(f"[{simulation_time}] R{self.id} LIGHT -> {new_color} (from {self.current_light})")
            
            # If using TwoRobot algorithm, ensure proper color tracking
            if self.algorithm == Algorithm.TWO_ROBOT:
                # Update both current and true final color
                self.current_light = new_color
                self.true_final_color = new_color
                
                # Log additional debug info
                Robot._logger.info(f"[{simulation_time}] R{self.id} TwoRobot color change:")
                Robot._logger.info(f"  - current_light: {self.current_light}")
                Robot._logger.info(f"  - true_final_color: {self.true_final_color}")
                Robot._logger.info(f"  - light_changes_count: {self.light_changes_count}")
                return
                
            # If using ColorBased algorithm, keep the innate color for display
            if self.algorithm == Algorithm.COLOR_BASED:
                # Update the internal color for algorithm logic
                self.current_light = new_color
                # Don't change the visible color for display
                return
            else:
                # For regular algorithms and MutualVisibility, update the visible color
                self.current_light = new_color

    def log_light_event(self, new_color: str, simulation_time: float) -> None:
        Robot._logger.info(f"[{simulation_time}] R{self.id} LIGHT -> {new_color}")

    def get_algorithm_for_color(self) -> Algorithm:
        """
        Returns the algorithm to use based on the robot's innate color.
        Different colored robots run different algorithms.
        """
        # For the ColorBased algorithm, we want to make the distinction very clear:
        # - Red robots do Gathering
        # - Blue robots do SEC
        if self.innate_color == "red":
            return Algorithm.GATHERING
        elif self.innate_color == "blue":
            return Algorithm.SEC
        
        # If color isn't one of the two main colors, use ID to determine algorithm
        # (even IDs do Gathering, odd IDs do SEC)
        return Algorithm.GATHERING if self.id % 2 == 0 else Algorithm.SEC

    def look(self, snapshot: dict[Id, SnapshotDetails], time: float) -> None:
        """
        Robot enters LOOK state:
          - Set state to LOOK (without changing light color for TwoRobot algorithm)
          - Build a 'visible' snapshot
          - Possibly terminate if alone
          - Compute next destination via chosen algorithm
        """
        # Store the current simulation time for use in algorithms
        self.snapshot_time = time
    
        # Fix for TwoRobot algorithm: don't change light color based on state
        if self.algorithm == Algorithm.TWO_ROBOT:
            self.state = RobotState.LOOK
            # Don't set light to blue for TwoRobot algorithm
            Robot._logger.info(f"[{time}] R{self.id} LOOK state (keeping light color {self.current_light})")
        else:
            # Original behavior for other algorithms
            self.set_light("blue", time)  
            self.state = RobotState.LOOK
    
        # Build the subset of snapshot that is visible to this robot
        # Optimized to reuse previous visibility calculations when appropriate
        self.snapshot = {}
    
        # Track if surrounding robots have significantly moved since last check
        significant_environment_change = False
        if self.last_visibility_update_time > 0:
            # Check if any robot has moved significantly since last visibility update
            for rid, snap in snapshot.items():
                if rid in self.visible_robots_cache:
                    # This robot was visible before - has it moved?
                    if hasattr(self, 'last_visible_positions') and rid in self.last_visible_positions:
                        last_pos = self.last_visible_positions[rid]
                        dist = math.dist(last_pos, snap.pos)
                        if dist > 10**-(self.threshold_precision + 2):  # A small threshold for movement
                            significant_environment_change = True
                            break
        
            # If positions haven't changed much and it hasn't been too long since last update
            if not significant_environment_change and time - self.last_visibility_update_time < 2.0:
                # Reuse previous visibility calculations
                for rid, snap in snapshot.items():
                    if rid in self.visible_robots_cache:
                        self.snapshot[rid] = SnapshotDetails(
                            self._convert_coordinate(snap.pos),
                            snap.state,
                            snap.frozen,
                            snap.terminated,
                            snap.multiplicity,
                            snap.light
                        )
            else:
                # Need to recalculate visibility
                self._recalculate_visibility(snapshot, time)
        else:
            # First time or reset - calculate visibility
            self._recalculate_visibility(snapshot, time)

        Robot._logger.info(
            f"[{time}] R{self.id} LOOK    -- Visible: {len(self.snapshot)} robots"
        )

        # If alone in the snapshot, we can consider ourselves converged/terminated
        # CRITICAL: Don't change color to gray for TwoRobot algorithm
        if len(self.snapshot) == 1:
            self.frozen = True
            self.terminated = True
            if self.algorithm != Algorithm.TWO_ROBOT:
                self.set_light("gray", time)
            self.wait(time)
            return

        # Select the algorithm and run compute
        algo_func, term_func = self._select_algorithm()
        self.calculated_position = self._compute(algo_func, term_func)

        dist_str = ""
        if self.calculated_position:
            dist_to_target = math.dist(self.coordinates, self.calculated_position)
            dist_str = f"Dist to target: {dist_to_target:.6f}"
    
            # Robot should only be considered frozen if it's at destination AND in the terminal state
            # or if the algorithm explicitly wants it not to move (staying in place)
            if dist_to_target < 10**-self.threshold_precision:
                if self.terminated:
                    self.frozen = True
                    Robot._logger.info(f"Robot {self.id} is frozen (reached destination and terminated)")
                else:
                    # For TwoRobot algorithm, robots can be at destination but should still move
                    # if their colors change - don't mark as frozen unless terminated
                    self.frozen = False
                    Robot._logger.info(f"Robot {self.id} at destination but not frozen")
        
                # For SEC robots running ColorBased, calculate SEC if needed
                if (self.algorithm == Algorithm.SEC or 
                    (self.algorithm == Algorithm.COLOR_BASED and 
                     self.get_algorithm_for_color() == Algorithm.SEC)):
                    if not self.sec:
                        # Force SEC calculation if not already done
                        _, extras = self._smallest_enclosing_circle()
                        if extras and extras[0]:
                            self.sec = extras[0]
                            Robot._logger.info(f"Robot {self.id} calculated SEC: {self.sec}")

        Robot._logger.info(
            f"[{time}] R{self.id} COMPUTE -> {self.calculated_position} | {dist_str}"
        )

        # If we are effectively at our computed position, skip the move
        if self.frozen:
            self.wait(time)
        else:
            self.frozen = False
    
    def _recalculate_visibility(self, snapshot: dict[Id, SnapshotDetails], time: float) -> None:
        """Helper to recalculate visibility from scratch"""
        self.visible_robots_cache.clear()
        self.last_visible_positions = {}
        
        for rid, snap in snapshot.items():
            if self._robot_is_visible(snap.pos):
                self.visible_robots_cache.add(rid)
                self.last_visible_positions[rid] = snap.pos
                self.snapshot[rid] = SnapshotDetails(
                    self._convert_coordinate(snap.pos),
                    snap.state,
                    snap.frozen,
                    snap.terminated,
                    snap.multiplicity,
                    snap.light
                )
        
        self.last_visibility_update_time = time

    def move(self, start_time: float) -> None:
        """
        Robot enters MOVE state:
          - Update state to MOVE (without changing light for TwoRobot algorithm)
          - STRICT ENFORCEMENT: BLACK robots in TwoRobot algorithm NEVER move
          - Record start_time, for interpolation in get_position.
        """
        # Enhanced debugging for movement issues
        self._logger.info(f"[{start_time}] R{self.id} MOVE method called - color: {self.current_light}, pos: {self.coordinates}, target: {self.calculated_position}")
        
        # CRITICAL FIX: BLACK robots in TwoRobot algorithm NEVER move
        if self.algorithm == Algorithm.TWO_ROBOT and self.current_light == "black":
            self._logger.info(f"[{start_time}] R{self.id} is BLACK in TwoRobot algorithm - NOT MOVING")
            self.calculated_position = self.coordinates  # Reset calculated position to current
            self.wait(start_time)  # Skip directly to WAIT state
            return
        
        # Debug: Calculate planned movement distance
        planned_distance = math.dist(self.coordinates, self.calculated_position)
        self._logger.info(f"[{start_time}] R{self.id} planning to move distance: {planned_distance:.6f}")
        
        # Fix for TwoRobot algorithm: don't change light color based on state
        if self.algorithm == Algorithm.TWO_ROBOT:
            self.state = RobotState.MOVE
            Robot._logger.info(f"[{start_time}] R{self.id} MOVE state (keeping light color {self.current_light})")
        else:
            # Original behavior for other algorithms
            self.set_light("red", start_time)
            self.state = RobotState.MOVE
        
        self.start_time = start_time
        
        # CRITICAL FIX: Use current coordinates as start position, not initial position
        self.start_position = self.coordinates
        
        # Check if we need to enforce movement
        if self.algorithm == Algorithm.TWO_ROBOT and self.current_light == "white" and planned_distance < 1e-10:
            # White robots should move even if the target is very close
            self._logger.info(f"[{start_time}] R{self.id} (WHITE) target too close - adding offset to ensure movement")
            # Add a slight offset to ensure some movement
            offset_x = 0.1 * (1 if self.id % 2 == 0 else -1)
            offset_y = 0.1 * (1 if self.id % 2 == 1 else -1)
            self.calculated_position = Coordinates(
                self.coordinates.x + offset_x, 
                self.coordinates.y + offset_y
            )
        
        # Clear position interpolation cache when starting a new move
        if hasattr(self, 'interpolation_cache'):
            self.interpolation_cache.clear()
        
        Robot._logger.info(f"[{start_time}] R{self.id} MOVE -> start pos = {self.start_position}, target = {self.calculated_position}")

    def wait(self, time: float) -> None:
        """
        Robot enters WAIT state:
          - Update state to WAIT (without changing light for TwoRobot algorithm)
          - finalize position
          - accumulate distance traveled
        """
        # Get final position BEFORE changing state
        final_position = self.get_position(time)
        
        # Fix for TwoRobot algorithm: don't change light color based on state
        if self.algorithm == Algorithm.TWO_ROBOT:
            self.state = RobotState.WAIT
            Robot._logger.info(f"[{time}] R{self.id} WAIT state (keeping light color {self.current_light})")
        else:
            # Original behavior for other algorithms
            self.set_light("green", time)
            self.state = RobotState.WAIT
        
        # Calculate actual distance moved from start position
        dist_moved = math.dist(self.start_position, final_position)
        
        # Update coordinates with the final position
        self.coordinates = final_position
        
        self.end_time = time
        self.travelled_distance += dist_moved
        
        # Track significant movement
        if dist_moved > 10**-(self.threshold_precision + 1):
            self.last_significant_movement_time = time
            self.movement_since_last_check = dist_moved

        Robot._logger.info(
            f"[{time}] R{self.id} WAIT -> moved {dist_moved:.5f}, total dist {self.travelled_distance:.5f}, new position {self.coordinates}"
        )
        # reset times
        self.start_time = None
        self.end_time = None
        # Clear interpolation cache
        self.interpolation_cache.clear()

    def get_position(self, time: float) -> Coordinates:
        """
        Interpolate position based on current state and movement constraints.
        Optimized with caching for better performance.
        """
        if self.state != RobotState.MOVE:
            return self.coordinates
        
        # Use cached position if available for this exact time
        cache_key = round(time, 6)  # Round to microsecond precision
        if cache_key in self.interpolation_cache:
            return self.interpolation_cache[cache_key]

        # Calculate total distance to destination
        dist_total = math.dist(self.start_position, self.calculated_position)
    
        # SAFETY CHECK: If distance is essentially zero, return current position
        if dist_total < 1e-10:
            return self.coordinates
    
        # Calculate elapsed time since movement started
        elapsed = time - (self.start_time or 0.0)
    
        # Calculate distance covered based on speed and time
        dist_covered = self.speed * elapsed
    
        result_position = None
        
        # Full rigid movement - go directly to destination
        if self.rigid_movement:
            if dist_covered >= dist_total - Robot._EPSILON_DISTANCE:
                result_position = self.calculated_position
            else:
                t = dist_covered / dist_total
                result_position = self._interpolate(self.start_position, self.calculated_position, t)
    
        # Non-rigid movement implementation
        else:
            # IMPORTANT FIX: Log the actual position with correct coordinates
            self._logger.info(f"Robot {self.id} moving from {self.start_position} toward {self.calculated_position}")
        
            # Minimum movement distance (can be configurable as a property)
            min_movement = max(0.1, self.speed * 0.1)  # At least 10% of speed
        
            # If target is close, always reach it
            if dist_total <= min_movement:
                result_position = self.calculated_position
            else:
                # If we've moved enough to reach destination, go there
                if dist_covered >= dist_total - Robot._EPSILON_DISTANCE:
                    result_position = self.calculated_position
                else:
                    # Calculate how far to move - always move at least 50% of distance for TwoRobot
                    if self.algorithm == Algorithm.TWO_ROBOT:
                        # Move at least 50% for TwoRobot algorithm
                        actual_movement = max(dist_covered, dist_total * 0.5)
                    else:
                        # For other algorithms, use normal movement calculation
                        actual_movement = min(dist_covered, dist_total)
                
                    # Ensure minimum movement
                    if actual_movement < min_movement:
                        actual_movement = min_movement
                    
                    # Make sure we don't exceed the total distance
                    actual_movement = min(actual_movement, dist_total)
                
                    # Calculate t parameter for interpolation (0 to 1)
                    t = actual_movement / dist_total
                
                    # Update coordinates with interpolated position
                    result_position = self._interpolate(self.start_position, self.calculated_position, t)
    
        # Cache the calculated position
        self.interpolation_cache[cache_key] = result_position
        return result_position

    def _select_algorithm(self) -> tuple[Callable, Callable]:
        """
        Return (algorithm_func, termination_check_func) based on robot's color or algorithm setting.
        """
        # If algorithm is set to COLOR_BASED, select based on innate color
        if self.algorithm == Algorithm.COLOR_BASED:
            algo = self.get_algorithm_for_color()
            Robot._logger.info(f"Robot {self.id} with color {self.innate_color} using algorithm {algo.name}")
        else:
            algo = self.algorithm
            
        match algo:
            case Algorithm.GATHERING:
                return (
                    lambda: algorithms.midpoint_algorithm(self),
                    lambda c,a: algorithms.midpoint_terminal(self, c, a)
                )
            case Algorithm.SEC:
                return (self._smallest_enclosing_circle, self._sec_terminal)
            case Algorithm.CUSTOM:
                return (self._run_custom_alg, self._run_custom_term_code)
            case Algorithm.COLOR_BASED:
                return (
                    lambda: algorithms.color_based_algorithm(self),
                    lambda c,a: algorithms.color_based_terminal(self, c, a)
                )
            case Algorithm.TWO_ROBOT:
                return (
                    lambda: algorithms.two_robot_algorithm(self),
                    lambda c,a: algorithms.two_robot_terminal(self, c, a)
                )
            case Algorithm.MUTUAL_VISIBILITY:
                return (
                    lambda: algorithms.mutual_visibility_algorithm(self),
                    lambda c,a: algorithms.mutual_visibility_terminal(self, c, a)
                )

    def _compute(
        self,
        algo_func: Callable[[], tuple[Coordinates, list]],
        term_func: Callable[[Coordinates, list], bool],
    ) -> Coordinates:
        coord, extras = algo_func()
        # If the algorithm says "you're done," mark as terminated (gray)
        if term_func and term_func(coord, extras):
            self.terminated = True
            # CRITICAL: Don't change color to gray for TwoRobot algorithm
            if self.algorithm != Algorithm.TWO_ROBOT:
                self.set_light("gray", self.start_time if self.start_time else 0.0)
        return coord

    def _convert_coordinate(self, coord: Coordinates) -> Coordinates:
        """
        Apply orientation transforms (rotation, reflection) to coordinates.
        Properly handles all transformation cases based on robot orientation.
        """
        if not self.orientation:
            return coord
            
        # Original coordinates
        x, y = coord.x, coord.y
        
        # Apply translation if specified
        if hasattr(self.orientation, 'translation') and self.orientation.translation:
            tx, ty = self.orientation.translation
            x += tx
            y += ty
            
        # Apply rotation if specified (in degrees)
        if hasattr(self.orientation, 'rotation') and self.orientation.rotation:
            angle_rad = math.radians(self.orientation.rotation)
            cos_theta = math.cos(angle_rad)
            sin_theta = math.sin(angle_rad)
            x_rot = x * cos_theta - y * sin_theta
            y_rot = x * sin_theta + y * cos_theta
            x, y = x_rot, y_rot
            
        # Apply reflection if specified
        if hasattr(self.orientation, 'reflection') and self.orientation.reflection:
            reflection_type = self.orientation.reflection
            if reflection_type == 'x':
                # Reflect across x-axis
                y = -y
            elif reflection_type == 'y':
                # Reflect across y-axis
                x = -x
            elif reflection_type == 'origin':
                # Reflect through origin
                x, y = -x, -y
            elif reflection_type == 'y=x':
                # Reflect across y=x line
                x, y = y, x
                
        return Coordinates(x, y)

    def _robot_is_visible(self, pos: Coordinates) -> bool:
        """
        Return True if the robot sees 'pos' within visibility_radius, or if unlimited.
        Optimized with cached distance calculations and obstruction handling.
        """
        if self.visibility_radius is None:
            return True
            
        # Create a cache key based on positions
        cache_key = (round(self.coordinates.x, 4), round(self.coordinates.y, 4), 
                     round(pos.x, 4), round(pos.y, 4))
                     
        # Check cache for this calculation
        if cache_key in Robot._calculation_cache:
            distance = Robot._calculation_cache[cache_key]
        else:
            distance = math.dist(self.coordinates, pos)
            # Store in cache (limit cache size to prevent memory issues)
            if len(Robot._calculation_cache) < 10000:
                Robot._calculation_cache[cache_key] = distance
                
        # If beyond visibility radius, not visible
        if distance > self.visibility_radius:
            return False
            
        # If obstructed visibility is enabled, check for obstructions
        if self.obstructed_visibility and self.snapshot:
            # Check if any robot obstructs the line of sight
            for _, details in self.snapshot.items():
                if details.pos != pos and self._obstructs_view(details.pos, pos):
                    return False
                
        return True

    def _obstructs_view(self, obstacle_pos: Coordinates, target_pos: Coordinates) -> bool:
        """
        Check if obstacle obstructs the view to the target using line-of-sight check.
        
        Args:
            obstacle_pos: Position of potential obstructing robot
            target_pos: Position of target robot to check visibility to
            
        Returns:
            bool: True if obstacle obstructs the view, False otherwise
        """
        # Vector from observer to target
        v_target = (target_pos.x - self.coordinates.x, target_pos.y - self.coordinates.y)
        target_dist = math.sqrt(v_target[0]**2 + v_target[1]**2)
        
        # Vector from observer to obstacle
        v_obstacle = (obstacle_pos.x - self.coordinates.x, obstacle_pos.y - self.coordinates.y)
        obstacle_dist = math.sqrt(v_obstacle[0]**2 + v_obstacle[1]**2)
        
        # If obstacle is further than target, it can't obstruct
        if obstacle_dist >= target_dist:
            return False
            
        # Normalize vectors for directional comparison
        if target_dist > 0:
            v_target = (v_target[0]/target_dist, v_target[1]/target_dist)
        else:
            return False  # Target is at same position as self
            
        if obstacle_dist > 0:
            v_obstacle = (v_obstacle[0]/obstacle_dist, v_obstacle[1]/obstacle_dist)
        else:
            return False  # Obstacle is at same position as self
        
        # Get dot product to check alignment
        dot_product = v_target[0]*v_obstacle[0] + v_target[1]*v_obstacle[1]
        
        # If obstacle is approximately on the line to the target and close enough
        if dot_product > 0.98:  # Within ~11 degrees
            # Get the perpendicular distance from obstacle to the line of sight
            # using the cross-product magnitude
            cross = abs(v_target[0]*v_obstacle[1] - v_target[1]*v_obstacle[0])
            perpendicular_dist = cross * obstacle_dist
            
            # Consider the obstacle radius (use robot size * 0.5 as approximation)
            robot_radius = 0.5  # Half of standard robot size unit
            
            # If perpendicular distance is less than effective robot size
            return perpendicular_dist < robot_radius
        
        return False

    def _interpolate(self, start: Coordinates, end: Coordinates, t: float) -> Coordinates:
        """Linearly interpolate between two points with parameter t (0-1) with safety checks"""
        # Safety check for invalid t values
        if not (0 <= t <= 1):
            t = max(0, min(1, t))  # Clamp to [0,1]
    
        # Safety check for identical points
        if math.isclose(start.x, end.x, abs_tol=1e-10) and math.isclose(start.y, end.y, abs_tol=1e-10):
            return Coordinates(start.x, start.y)
            
        # Check cache for this interpolation 
        cache_key = (round(start.x, 4), round(start.y, 4), 
                     round(end.x, 4), round(end.y, 4), round(t, 4))
                 
        if cache_key in Robot._calculation_cache:
            return Robot._calculation_cache[cache_key]
            
        result = Coordinates(
            start.x + t*(end.x - start.x),
            start.y + t*(end.y - start.y)
        )
    
        # Store in cache
        if len(Robot._calculation_cache) < 10000:
            Robot._calculation_cache[cache_key] = result
        
        return result

    #
    # -------------------- CUSTOM CODE  --------------------
    #
    def _run_custom_alg(self):
        scope = {
            'Coordinates': Coordinates,
            'self': self,
            'output': (Coordinates(0,0), [])
        }
        exec(self.custom_alg, scope)
        return scope['output']

    def _run_custom_term_code(self, coord: Coordinates, args: list):
        scope = {
            'coord': coord,
            'args': args,
            'Coordinates': Coordinates,
            'self': self,
            'output': False
        }
        exec(self.custom_term_code, scope)
        return scope['output']

    #
    # -----------------  SEC ALGORITHM ---------------------
    #
    def _smallest_enclosing_circle(self) -> tuple[Coordinates, list[Circle]]:
        """
        Basic version of the SEC logic, stored here as an example.
        Optimized with caching for repeated calculations.
        """
        visible_ids = self._get_visible_robots()
        num = len(self.snapshot)
        if num == 0:
            return (Coordinates(0,0), [None])
        elif num == 1:
            return (list(self.snapshot.values())[0].pos, [None])
        elif num == 2:
            i, j = visible_ids[0], visible_ids[1]
            A = self.snapshot[i].pos
            B = self.snapshot[j].pos
            
            # Check if we've already calculated a circle for these points
            circle_key = tuple(sorted([
                (round(A.x, 4), round(A.y, 4)), 
                (round(B.x, 4), round(B.y, 4))
            ]))
            
            if circle_key in Robot._calculation_cache:
                self.sec = Robot._calculation_cache[circle_key]
            else:
                self.sec = self._circle_from_two(A,B)
                if len(Robot._calculation_cache) < 10000:
                    Robot._calculation_cache[circle_key] = self.sec
                
            return (self._closest_point_on_circle(self.sec, self.coordinates), [self.sec])
        else:
            # Check if we already have an SEC solution for the current robot positions
            # Only use if exact same set of robots with same positions
            if hasattr(self, 'last_sec_positions') and self.last_sec_positions:
                same_positions = True
                if len(visible_ids) == len(self.last_sec_positions):
                    for id in visible_ids:
                        if id not in self.last_sec_positions:
                            same_positions = False
                            break
                        pos = self.snapshot[id].pos
                        last_pos = self.last_sec_positions[id]
                        if math.dist(pos, last_pos) > 10**-(self.threshold_precision+1):
                            same_positions = False
                            break
                
                    if same_positions and self.sec:
                        # Positions haven't changed significantly, reuse solution
                        return (self._closest_point_on_circle(self.sec, self.coordinates), [self.sec])
            
            # Need to recalculate SEC
            self.sec = self._sec_welzl(visible_ids)
            
            # Store positions for future checks
            self.last_sec_positions = {}
            for id in visible_ids:
                self.last_sec_positions[id] = self.snapshot[id].pos
                
            return (self._closest_point_on_circle(self.sec, self.coordinates), [self.sec])

    def _sec_terminal(self, _, args: list[Circle]) -> bool:
        """
        Check if all visible robots lie on the final circle. If so, we are done.
        """
        circle = args[0]
        if not circle:
            return True
        for rid in self._get_visible_robots():
            if not self._is_point_on_circle(self.snapshot[rid].pos, circle):
                return False
        return True

    def _get_visible_robots(self):
        return sorted(self.snapshot.keys())

    def _circle_from_two(self, a: Coordinates, b: Coordinates) -> Circle:
        center = Coordinates((a.x + b.x)/2, (a.y + b.y)/2)
        radius = math.dist(a,b)/2
        return Circle(center, radius)

    def _closest_point_on_circle(self, circle: Circle, point: Coordinates) -> Coordinates:
        """Find the closest point on a circle to the given point"""
        # Check cache for this calculation
        cache_key = (round(circle.center.x, 4), round(circle.center.y, 4), round(circle.radius, 4),
                     round(point.x, 4), round(point.y, 4))
                     
        if cache_key in Robot._calculation_cache:
            return Robot._calculation_cache[cache_key]
            
        center = circle.center
        r = circle.radius
        d = math.dist(center, point)
        if d == 0:
            # Point is at center, return any point on circle
            result = Coordinates(center.x + r, center.y)
        else:
            scale = r/d
            result = Coordinates(
                center.x + (point.x - center.x)*scale,
                center.y + (point.y - center.y)*scale
            )
        
        # Store in cache
        if len(Robot._calculation_cache) < 10000:
            Robot._calculation_cache[cache_key] = result
            
        return result

    def _sec_welzl(self, robot_ids):
        # randomize
        Robot._generator.shuffle(robot_ids)
        return self._sec_welzl_recur(robot_ids, [], len(robot_ids))

    def _sec_welzl_recur(self, points, R, n):
        if n == 0 or len(R) == 3:
            return self._min_circle(R)
        idx = Robot._generator.integers(0, n) if n>0 else 0
        p = self.snapshot[points[idx]].pos
        points[idx], points[n-1] = points[n-1], points[idx]
        c = self._sec_welzl_recur(points, R.copy(), n-1)
        if math.dist(p, c.center) <= c.radius + (10**-self.threshold_precision):
            return c
        R.append(p)
        return self._sec_welzl_recur(points, R.copy(), n-1)

    def _min_circle(self, coords_list):
        if not coords_list:
            return Circle(Coordinates(0,0), 0)
        if len(coords_list) == 1:
            return Circle(coords_list[0], 0)
        if len(coords_list) == 2:
            return self._circle_from_two(coords_list[0], coords_list[1])
        return self._circle_from_three(coords_list[0], coords_list[1], coords_list[2])

    def _circle_from_three(self, a: Coordinates, b: Coordinates, c: Coordinates) -> Circle:
        """Calculate the circle passing through three points"""
        # Check cache for this calculation
        points = sorted([
            (round(a.x, 4), round(a.y, 4)),
            (round(b.x, 4), round(b.y, 4)),
            (round(c.x, 4), round(c.y, 4))
        ])
        cache_key = tuple(points)
        
        if cache_key in Robot._calculation_cache:
            return Robot._calculation_cache[cache_key]
            
        D = 2*(a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y))
        if abs(D) < 1e-15:
            # fallback
            result = self._circle_from_two(a,b)
        else:
            ux = ((a.x**2 + a.y**2)*(b.y - c.y) +
                (b.x**2 + b.y**2)*(c.y - a.y) +
                (c.x**2 + c.y**2)*(a.y - b.y)) / D
            uy = ((a.x**2 + a.y**2)*(c.x - b.x) +
                (b.x**2 + b.y**2)*(a.x - c.x) +
                (c.x**2 + c.y**2)*(b.x - a.x)) / D
            center = Coordinates(ux, uy)
            radius = math.dist(center, a)
            result = Circle(center, radius)
        
        # Store in cache
        if len(Robot._calculation_cache) < 10000:
            Robot._calculation_cache[cache_key] = result
            
        return result

    def _is_point_on_circle(self, p: Coordinates, c: Circle) -> bool:
        """Check if a point lies on a circle within precision threshold"""
        # Use cached distance if available
        cache_key = (round(p.x, 4), round(p.y, 4), 
                    round(c.center.x, 4), round(c.center.y, 4))
                    
        if cache_key in Robot._calculation_cache:
            dist = Robot._calculation_cache[cache_key]
        else:
            dist = math.dist(p, c.center)
            # Store in cache
            if len(Robot._calculation_cache) < 10000:
                Robot._calculation_cache[cache_key] = dist
                
        return abs(dist - c.radius) < 10**-self.threshold_precision

    def __str__(self):
        return (f"R{self.id}, speed: {self.speed}, state: {self.state}, "
                f"color: {self.current_light}, coords: {self.coordinates}")

    def mutual_visibility(self):
        """Mutual visibility algorithm implementation"""
        # Get all other robots
        other_robots = [r for r in self.scheduler.robots if r != self]
        
        if not other_robots:
            return
            
        # Calculate center of mass of all robots
        center_x = sum(r.x for r in other_robots) / len(other_robots)
        center_y = sum(r.y for r in other_robots) / len(other_robots)
        
        # Calculate desired spacing between robots
        desired_spacing = 2.0  # Increased from 1.0 for better visibility
        
        # Calculate target position
        # Move towards the center while maintaining spacing
        target_x = center_x
        target_y = center_y
        
        # Add some randomness to prevent robots from getting stuck
        target_x += random.uniform(-0.1, 0.1)
        target_y += random.uniform(-0.1, 0.1)
        
        # Calculate movement vector
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Normalize movement vector
        distance = math.sqrt(dx*dx + dy*dy)
        if distance > 0:
            dx = dx / distance
            dy = dy / distance
            
            # Apply movement with reduced speed for smoother convergence
            self.x += dx * 0.5
            self.y += dy * 0.5
            
            # Ensure robots stay within bounds
            self.x = max(0, min(self.x, self.scheduler.width))
            self.y = max(0, min(self.y, self.scheduler.height))
