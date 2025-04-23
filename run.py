import sys
import json
from robot import RobotState, Algorithm, Coordinates # Only need these specifics here
from scheduler import Scheduler # Import the main class
import numpy as np

# --- Global state for the simulation ---
scheduler_instance: Scheduler|None = None
simulation_params = {}
is_running = False
current_simulation_time = 0.0
max_simulation_time = 1000.0 # Add a safety break
event_count = 0
max_events = 10000 # Add safety break

# Simple logging
def log_info(msg):
    print(f"INFO: {msg}")

def log_error(msg):
    print(f"ERROR: {msg}", file=sys.stderr)

# --- Simulation Control Functions (Callable from JS) ---

def setup_simulation(params_json_str):
    """Initializes the scheduler with parameters from JS."""
    global scheduler_instance, simulation_params, is_running, current_simulation_time
    global event_count
    log_info("Setting up simulation...")
    try:
        params = json.loads(params_json_str)
        simulation_params = params
        log_info(f"Params: {params}")

        # Generate initial positions if not provided
        num_robots = int(params.get("num_of_robots", 5))
        initial_positions = params.get("initial_positions") # Expect list of [x,y]

        if not initial_positions or len(initial_positions) != num_robots:
            log_info("Generating random initial positions...")
            width = float(params.get("width_bound", 100))
            height = float(params.get("height_bound", 100))
            seed = int(params.get("random_seed", 12345)) # Use seed from params
            generator = np.random.default_rng(seed=seed) # Use separate generator for init pos?
            x_positions = generator.uniform(low=-width / 2, high=width / 2, size=(num_robots,))
            y_positions = generator.uniform(low=-height / 2, high=height / 2, size=(num_robots,))
            initial_positions = np.column_stack((x_positions, y_positions)).tolist()
            log_info(f"Generated positions: {initial_positions}")


        # Create Scheduler instance
        scheduler_instance = Scheduler(
            seed=int(params.get("random_seed", 12345)),
            num_of_robots=num_robots,
            initial_positions=initial_positions,
            robot_speeds=float(params.get("robot_speeds", 1.0)),
            rigid_movement=bool(params.get("rigid_movement", True)),
            threshold_precision=int(params.get("threshold_precision", 5)),
            sampling_rate=float(params.get("sampling_rate", 0.1)),
            labmda_rate=float(params.get("lambda_rate", 5.0)),
            algorithm=params.get("algorithm", Algorithm.GATHERING), # Use string 'Gathering' or 'SEC'
            visibility_radius=params.get("visibility_radius"), # Pass None or float
            num_of_faults=int(params.get("num_of_faults", 0)),
            multiplicity_detection=True # Enable multiplicity always for now? Or add checkbox?
        )

        is_running = True
        current_simulation_time = 0.0
        event_count = 0
        log_info("Scheduler initialized successfully.")
        # Return the very initial state
        initial_state = {
            "status": "initialized",
            "time": 0.0,
            "robots": scheduler_instance.get_all_robot_data_for_js(),
             "message": "Simulation Initialized"
        }
        return json.dumps(initial_state)

    except Exception as e:
        log_error(f"Error setting up simulation: {e}")
        is_running = False
        error_state = {
            "status": "error",
            "time": 0.0,
            "robots": [],
             "message": f"Setup Error: {e}"
        }
        # Propagate error back to JS
        # Option 1: Return JSON with error
        return json.dumps(error_state)
        # Option 2: Raise exception (Pyodide might catch and pass to JS .catch())
        # raise e


def run_simulation_step():
    """Executes one event from the scheduler queue."""
    global scheduler_instance, is_running, current_simulation_time, event_count
    if not scheduler_instance or not is_running:
        return json.dumps({"status": "idle", "message": "Simulation not running."})

    try:
        # Safety break conditions
        if current_simulation_time > max_simulation_time:
            log_info(f"Simulation stopped: Max time ({max_simulation_time}) reached.")
            is_running = False
            return json.dumps({"status": "ended", "time": current_simulation_time, "robots": scheduler_instance.get_all_robot_data_for_js(), "message": "Ended: Max time reached"})

        event_count += 1
        if event_count > max_events:
            log_info(f"Simulation stopped: Max events ({max_events}) reached.")
            is_running = False
            return json.dumps({"status": "ended", "time": current_simulation_time, "robots": scheduler_instance.get_all_robot_data_for_js(), "message": "Ended: Max events reached"})


        exit_code, time, snapshot_data = scheduler_instance.handle_event()
        current_simulation_time = time # Update global time

        status = "running"
        message = f"Event handled (code {exit_code})"

        if scheduler_instance.terminate or exit_code == -1:
            is_running = False
            status = "ended"
            message = "Simulation Ended (Terminated)"
            log_info("Simulation terminated by scheduler.")


        # Prepare data for JS
        step_result = {
            "status": status,
            "time": current_simulation_time,
            # Use the dedicated function to get robot data in JS-friendly format
            "robots": scheduler_instance.get_all_robot_data_for_js(),
            "exit_code": exit_code,
            "message": message
            # Include snapshot if needed for debugging?
            # "snapshot": snapshot_data # This snapshot might be slightly outdated vs get_all_robot_data
        }
        return json.dumps(step_result)

    except Exception as e:
        log_error(f"Error during simulation step: {e}")
        is_running = False
        # Maybe include stack trace if possible?
        import traceback
        error_state = {
            "status": "error",
            "time": current_simulation_time,
            "robots": scheduler_instance.get_all_robot_data_for_js() if scheduler_instance else [],
             "message": f"Runtime Error: {e}\n{traceback.format_exc()}"
        }
        return json.dumps(error_state)


def stop_simulation():
    """Stops the simulation loop."""
    global is_running, scheduler_instance
    log_info("Stopping simulation...")
    is_running = False
    # Maybe clean up scheduler instance?
    # scheduler_instance = None
    stopped_state = {
        "status": "stopped",
        "time": current_simulation_time,
        "robots": scheduler_instance.get_all_robot_data_for_js() if scheduler_instance else [],
        "message": "Simulation stopped by user."
    }
    return json.dumps(stopped_state)


log_info("run.py loaded.")