import json
import socket
from enums import Algorithm
from scheduler import Scheduler
import numpy as np
import logging
from flask import Flask, jsonify, request, Response, send_from_directory
import webbrowser
import threading
import json
from flask_socketio import SocketIO, emit
from datetime import datetime
import os
import time
import gc
import random
import argparse

# Add command line argument parsing
def parse_args():
    parser = argparse.ArgumentParser(description='Run robot simulation')
    parser.add_argument('--algorithm', type=str, default='TwoRobot', help='Algorithm to use')
    parser.add_argument('--num_of_robots', type=int, default=2, help='Number of robots')
    parser.add_argument('--random_seed', type=int, default=3000000001, help='Random seed')
    parser.add_argument('--width_bound', type=float, default=500, help='Width bound')
    parser.add_argument('--height_bound', type=float, default=500, help='Height bound')
    parser.add_argument('--robot_speeds', type=float, default=1.0, help='Robot speeds')
    parser.add_argument('--rigid_movement', action='store_true', help='Rigid movement')
    parser.add_argument('--threshold_precision', type=int, default=5, help='Threshold precision')
    parser.add_argument('--sampling_rate', type=float, default=1.0, help='Sampling rate')
    parser.add_argument('--lambda_rate', type=float, default=10.0, help='Lambda rate')
    parser.add_argument('--visibility_radius', type=float, default=200.0, help='Visibility radius')
    parser.add_argument('--obstructed_visibility', action='store_true', help='Obstructed visibility')
    return parser.parse_args()

def get_log_name():
    date = datetime.now()
    milliseconds = date.microsecond // 1000
    return f"{date.year}-{date.month}-{date.day}-{date.hour}-{date.minute}-{date.second}-{milliseconds}.txt"

def setup_logger(simulation_id, algo_name):
    logger = logging.getLogger(f"app_{simulation_id}")
    logger.setLevel(logging.INFO)

    log_dir = f"./logs/{algo_name}/"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{get_log_name()}")

    new_file_handler = logging.FileHandler(log_file)
    formatter = logging.Formatter("")
    new_file_handler.setFormatter(formatter)
    logger.addHandler(new_file_handler)

    return logger

def generate_initial_positions(generator, width_bound, height_bound, n):
    x_positions = generator.uniform(low=-width_bound, high=height_bound, size=(n,))
    y_positions = generator.uniform(low=-height_bound, high=height_bound, size=(n,))
    positions = np.column_stack((x_positions, y_positions))
    return positions

# Batch update mechanism to reduce frontend overhead
class BatchUpdater:
    def __init__(self, socket, batch_size=3, max_delay=0.15):
        self.socket = socket
        self.batch_size = batch_size
        self.max_delay = max_delay
        self.batch = []
        self.last_emit_time = 0
        self.lock = threading.Lock()
        
    def add_snapshot(self, snapshot_data):
        """Add a snapshot to the batch"""
        with self.lock:
            self.batch.append(snapshot_data)
            
            current_time = time.time()
            elapsed = current_time - self.last_emit_time
            
            # Send if batch is full or enough time has elapsed
            if len(self.batch) >= self.batch_size or (self.batch and elapsed >= self.max_delay):
                self._emit_batch()
                
    def _emit_batch(self):
        """Emit the current batch to the client"""
        if not self.batch:
            return
            
        # For small batches, send individual updates
        if len(self.batch) <= 1:
            for snapshot in self.batch:
                self.socket.emit(
                    "simulation_data",
                    json.dumps(snapshot)
                )
        else:
            # Send as a batch update
            self.socket.emit(
                "simulation_data_batch",
                json.dumps({
                    "batch": self.batch
                })
            )
            
        self.batch = []
        self.last_emit_time = time.time()
        
    def flush(self):
        """Flush any remaining snapshots"""
        with self.lock:
            self._emit_batch()

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

app = Flask(__name__, 
           static_folder="static",
           static_url_path="",
           template_folder="static")
socketio = SocketIO(app, 
                   async_mode='threading',
                   cors_allowed_origins="*",
                   logger=True,
                   engineio_logger=True,
                   ping_timeout=60,
                   ping_interval=25)

simulation_thread = None
terminate_flag = False
simulation_id = 0
logger = None
batch_updater = BatchUpdater(socketio)

# Add memory logging if enabled
enable_memory_logging = False
memory_log_interval = 10  # seconds
last_memory_log = 0

# Add error handlers
@app.errorhandler(404)
def not_found_error(error):
    return "Not found", 404

@app.errorhandler(500)
def internal_error(error):
    return "Internal server error", 500

# Ensure proper response handling
@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response

@socketio.on("start_simulation")
def handle_simulation_request(data):
    global simulation_thread, terminate_flag, logger, batch_updater, simulation_id

    try:
        # Get simulation ID from request data
        simulation_id = data.get("simulation_id", str(int(time.time())))
        logger = setup_logger(simulation_id, data.get("algorithm", "Unknown"))
        logger.info(f"Starting simulation with ID: {simulation_id}")

        # Terminate existing simulation thread if needed
        if simulation_thread and simulation_thread.is_alive():
            logger.info("Simulation Interrupted... (A new simulation was requested)")
            terminate_flag = True
            batch_updater.flush()  # Flush any pending updates
            simulation_thread.join()

        terminate_flag = False
        
        # Create new batch updater for this simulation
        batch_updater = BatchUpdater(socketio, 
                                   batch_size=max(1, min(5, data.get("num_of_robots", 3) // 10)),
                                   max_delay=0.12)

        # Get the seed value and create random generator
        seed = data.get("random_seed", 3000000001)  # Default seed if not provided
        robot_colors = []  # Initialize robot_colors at the start
        
        # For TwoRobot algorithm, preserve the special seed values
        is_two_robot = data.get("algorithm") == "TwoRobot"
        if is_two_robot:
            # Define the special seed values for TwoRobot scenarios
            TWO_BLACK_SEED = 1000000001
            TWO_WHITE_SEED = 2000000001
            ONE_EACH_SEED = 3000000001
            
            # Log the seed value for debugging
            logger.info(f"Received seed value: {seed}")
            
            # Force exactly 2 robots
            num_robots = 2
            data["num_of_robots"] = 2
            
            # Get the scenario from the UI
            scenario = data.get("twoRobotScenario", "")
            logger.info(f"TwoRobot scenario selected: {scenario}")
            
            # Set robot colors based on the scenario
            if "Two Black" in scenario:
                robot_colors = ["black", "black"]
                logger.info("Setting both robots to BLACK (from scenario)")
            elif "Two White" in scenario:
                robot_colors = ["white", "white"]
                logger.info("Setting both robots to WHITE (from scenario)")
            elif "One Black, One White" in scenario:
                robot_colors = ["black", "white"]
                logger.info("Setting one robot BLACK, one WHITE (from scenario)")
            elif seed == TWO_BLACK_SEED:
                robot_colors = ["black", "black"]
                logger.info("Setting both robots to BLACK (from seed)")
            elif seed == TWO_WHITE_SEED:
                robot_colors = ["white", "white"]
                logger.info("Setting both robots to WHITE (from seed)")
            elif seed == ONE_EACH_SEED:
                robot_colors = ["black", "white"]
                logger.info("Setting one robot BLACK, one WHITE (from seed)")
            else:
                # Default case - use one black, one white
                robot_colors = ["black", "white"]
                logger.info(f"Using default colors: {robot_colors}")
            
            # Store colors in data for scheduler
            data["robot_colors"] = robot_colors
            logger.info(f"TwoRobot colors explicitly assigned: {robot_colors}")
            
            # Set robot speeds and sampling rate regardless of color
            data["sampling_rate"] = 0.05   # Faster updates for better responsiveness
            data["lambda_rate"] = 30.0     # Faster event generation
            data["robot_speeds"] = 5.0     # Ensure robots can move (CRITICAL)
            
            logger.info(f"Robot speeds set to: {data['robot_speeds']}")
            logger.info(f"Sampling rate set to: {data['sampling_rate']}")
            logger.info(f"Lambda rate set to: {data['lambda_rate']}")
            
            # Use the provided seed for position generation
            generator = np.random.default_rng(seed=seed)
            
            # Generate random positions for TwoRobot if none provided
            if len(data.get("initial_positions", [])) < 2:
                # Force positions to be more widely separated for better visibility
                # Generate the first position
                pos1 = [
                    generator.uniform(-data["width_bound"]/2, data["width_bound"]/2),
                    generator.uniform(-data["height_bound"]/2, data["height_bound"]/2)
                ]
            
                # Generate the second position at least 200 units away from the first
                while True:
                    pos2 = [
                        generator.uniform(-data["width_bound"]/2, data["width_bound"]/2),
                        generator.uniform(-data["height_bound"]/2, data["height_bound"]/2)
                    ]
                    # Calculate distance between positions
                    dist = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
                    if dist >= 200:  # Increased minimum separation for better visibility
                        break
            
                data["initial_positions"] = [pos1, pos2]
                logger.info(f"Generated TwoRobot positions: {data['initial_positions']}")
            
            data["random_seed"] = seed     # Preserve the seed value
        else:
            num_robots = data.get("num_of_robots", 2)  # Default to 2 if not specified
            generator = np.random.default_rng(seed=seed)

        # Get the initial_positions from data
        initial_positions = data.get("initial_positions", [])
        # Then check if it's empty
        if len(initial_positions) == 0:
            # Random positions
            initial_positions = generate_initial_positions(
                generator, data["width_bound"], data["height_bound"], num_robots
            )
            data["initial_positions"] = initial_positions.tolist()

        # If initial_positions is still empty and not user-defined, generate random positions
        if len(initial_positions) == 0:
            # Random positions
            initial_positions = generate_initial_positions(
                generator, data["width_bound"], data["height_bound"], num_robots
            )
            data["initial_positions"] = initial_positions.tolist()

        # For ColorBased algorithm, we need to assign colors
        is_color_based = data.get("algorithm") == "ColorBased"

        if not is_two_robot and len(initial_positions) != 0:
            # User defined
            num_robots = len(initial_positions)
            # Check if positions include color information
            if is_color_based and len(initial_positions[0]) >= 3:
                robot_colors = [pos[2] for pos in initial_positions]
                initial_positions = [[pos[0], pos[1]] for pos in initial_positions]
            else:
                # Assign colors automatically for ColorBased algorithm
                if is_color_based:
                    # First half red, second half blue
                    half = num_robots // 2
                    robot_colors = ["red"] * half + ["blue"] * (num_robots - half)
                    # Special case for just 2 robots - one red, one blue
                    if num_robots == 2:
                        robot_colors = ["red", "blue"]
        
        # Make sure we have colors for every robot
        if is_color_based and len(robot_colors) < num_robots:
            # Fill in any missing colors
            remaining = num_robots - len(robot_colors)
            robot_colors.extend(["red" if i % 2 == 0 else "blue" for i in range(remaining)])
        
        # Shuffle colors to avoid bias, except for TwoRobot algorithm
        if is_color_based and num_robots > 2 and not is_two_robot:
            generator.shuffle(robot_colors)
            logger.info(f"Robot colors assigned: {robot_colors}")
        elif is_two_robot:
            logger.info(f"TwoRobot colors confirmed: {robot_colors}")

        # Optimize sampling rate based on number of robots
        if num_robots > 20:
            # For large simulations, reduce sampling rate
            effective_sampling_rate = max(0.5, data.get("sampling_rate", 1.0) * 2)
        else:
            effective_sampling_rate = data.get("sampling_rate", 1.0)

        logger.info(f"Initial positions: {initial_positions}")
        logger.info(f"Number of robots: {num_robots}")
        
        try:
            scheduler = Scheduler(
                logger=logger,
                seed=seed,
                num_of_robots=num_robots,
                initial_positions=initial_positions,
                robot_speeds=data.get("robot_speeds", 1.0),
                rigid_movement=data.get("rigid_movement", False),
                threshold_precision=data.get("threshold_precision", 5),
                sampling_rate=effective_sampling_rate,
                lambda_rate=data.get("lambda_rate", 10.0),
                algorithm=data.get("algorithm", "TwoRobot"),
                custom_alg=data.get("custom_alg", ""),
                custom_term_code=data.get("custom_term_code", ""),
                visibility_radius=data.get("visibility_radius", 200.0),
                robot_colors=robot_colors if (is_color_based or is_two_robot) else None,
                obstructed_visibility=data.get("obstructed_visibility", False)
            )
            
            # Start simulation in a separate thread
            simulation_thread = threading.Thread(target=run_simulation, args=(scheduler,))
            simulation_thread.start()
            
            # Send initial response
            emit("simulation_started", {
                "simulation_id": simulation_id,
                "message": "Simulation started successfully"
            })
            
        except Exception as e:
            logger.error(f"Error creating scheduler: {str(e)}")
            emit("simulation_error", {
                "simulation_id": simulation_id,
                "error": str(e)
            })
            
    except Exception as e:
        logger.error(f"Error handling simulation request: {str(e)}")
        emit("simulation_error", {
            "simulation_id": simulation_id,
            "error": str(e)
        })

def get_memory_usage():
    """Get current memory usage information"""
    try:
        import psutil
        process = psutil.Process(os.getpid())
        mem_info = process.memory_info()
        mem_usage_bytes = mem_info.rss
        mem_usage_mb = mem_usage_bytes / (1024 * 1024)
        percent = process.memory_percent()
        return {
            "used_bytes": mem_usage_bytes,
            "used_mb": mem_usage_mb,
            "percent": percent
        }
    except ImportError:
        return {
            "used_bytes": 0,
            "used_mb": 0,
            "percent": 0
        }

@socketio.on("connect")
def client_connect():
    print("Client connected")
    socketio.emit("connection_status", "Connected successfully")

@socketio.on("disconnect")
def client_disconnect():
    print("Client disconnected")

# Add support for batched snapshot handling in the frontend
@socketio.on("batch_ack")
def handle_batch_acknowledgment(data):
    """Client acknowledges receipt of a snapshot batch"""
    # This can be used to implement flow control if needed
    pass

@app.route("/<path:path>")
def serve_static(path):
    try:
        return send_from_directory(app.static_folder, path)
    except Exception as e:
        print(f"Error serving static file {path}: {e}")
        return "File not found", 404

@app.route("/")
def serve_frontend():
    try:
        return send_from_directory(app.static_folder, "index.html")
    except Exception as e:
        print(f"Error serving frontend: {e}")
        return "Frontend not found", 404

def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) == 0

def open_browser(port):
    try:
        webbrowser.open(f"http://127.0.0.1:{port}/")
    except Exception as e:
        print(f"Could not open browser automatically. Please open http://127.0.0.1:{port}/ manually in your browser.")

# Find an available port
port = 8081
while is_port_in_use(port):
    port += 1

print(f"Starting server on port {port}")
print(f"Please open http://127.0.0.1:{port}/ in your browser")
print(f"Static folder path: {os.path.abspath(app.static_folder)}")
print(f"Available files in static folder: {os.listdir(app.static_folder)}")

def run_simulation(scheduler):
    global terminate_flag, last_memory_log

    try:
        last_memory_log = time.time()
        snapshot_buffer = []
        last_snapshot_time = 0
        snapshot_count = 0
        simulation_start_time = time.time()

        with app.app_context():
            socketio.emit("simulation_start", simulation_id)
            
            while not terminate_flag:
                exit_code = scheduler.handle_event()
                
                # Log memory usage if enabled
                if enable_memory_logging and time.time() - last_memory_log > memory_log_interval:
                    mem_usage = get_memory_usage()
                    logger.info(f"Memory usage: {mem_usage['used_mb']:.2f}MB, {mem_usage['percent']:.1f}%")
                    last_memory_log = time.time()
                
                current_time = time.time()
                elapsed_time = current_time - simulation_start_time
                
                if exit_code == 0:
                    # We handled a "sample" event. 
                    # If there's a new snapshot in scheduler.visualization_snapshots, emit it.
                    snapshots = scheduler.visualization_snapshots
                    if snapshots:
                        snapshot_count += 1
                        latest_snapshot = snapshots[-1]
                        
                        # For efficiency, only send every Nth snapshot when there are many robots
                        # or if enough time has passed since the last snapshot
                        skip_condition = False
                        
                        # For large numbers of robots, or long-running simulations, throttle updates
                        if scheduler.num_of_robots > 30:
                            # For very large simulations, be more selective
                            skip_condition = (snapshot_count % 3 != 0) and \
                                            (current_time - last_snapshot_time < 0.3)
                        elif scheduler.num_of_robots > 15:
                            # For medium simulations
                            skip_condition = (snapshot_count % 2 != 0) and \
                                            (current_time - last_snapshot_time < 0.2)
                        
                        # MutualVisibility algorithm: send updates less frequently
                        if scheduler.algorithm == "MutualVisibility":
                            skip_condition = snapshot_count % 5 != 0  # Send every 5th snapshot
                        
                        if not skip_condition or elapsed_time < 10.0:  # Always show initial snapshots
                            batch_updater.add_snapshot({
                                "simulation_id": simulation_id,
                                "snapshot": latest_snapshot,
                            })
                            last_snapshot_time = current_time

                # Give MutualVisibility algorithm more time (240s instead of 120s)
                if exit_code < 0 or (scheduler.algorithm == "MutualVisibility" and elapsed_time > 240.0):
                    break
            
            # Cleanup to help garbage collection
            if not terminate_flag:  # Normal termination
                scheduler.snapshot_history.clear()
                scheduler.visualization_snapshots.clear()
                scheduler.distance_cache.clear()
                scheduler.visibility_cache.clear()
                gc.collect()  # Explicitly run garbage collection to free memory
    except Exception as e:
        error_msg = f"Error in simulation thread: {str(e)}"
        logger.error(error_msg)
        socketio.emit("simulation_error", {
            "simulation_id": simulation_id,
            "error": error_msg
        })

if __name__ == "__main__":
    try:
        # Start the Flask server
        socketio.run(app, port=port, debug=False)
    except Exception as e:
        print(f"Error starting server: {e}")
        print("Please make sure no other application is using port 8081")
