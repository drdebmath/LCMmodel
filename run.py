import json
import socket
import numpy as np
import logging
from datetime import datetime
import os
import threading
import webbrowser
from typing import Dict, List, Optional, Union

from flask import Flask, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit
from enums import Algorithm
from scheduler import Scheduler

# Configuration Constants
MAX_SNAPSHOTS = 1000  # Maximum frames to keep in memory
DEFAULT_PORT = 8080
MAX_PORT_ATTEMPTS = 10
DEFAULT_FAULT_PROB = 0.3

# Configure Flask app
app = Flask(__name__, static_folder="static")
socketio = SocketIO(app)

# Global simulation state
simulation_thread = None
terminate_flag = False
simulation_id = 0
logger = None

# Disable Flask's default logging
logging.getLogger("werkzeug").setLevel(logging.ERROR)


def get_log_name() -> str:
    """Generate timestamped log filename with milliseconds precision"""
    date = datetime.now()
    milliseconds = date.microsecond // 1000
    return f"{date.year}-{date.month}-{date.day}-{date.hour}-{date.minute}-{date.second}-{milliseconds}.txt"


def setup_logger(simulation_id: int, algo_name: str) -> logging.Logger:
    """Configure logging for each simulation with directory creation"""
    logger = logging.getLogger(f"app_{simulation_id}")
    logger.setLevel(logging.INFO)

    log_dir = f"./logs/{algo_name}/"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, get_log_name())

    file_handler = logging.FileHandler(log_file)
    formatter = logging.Formatter("%(message)s")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger


def generate_initial_positions(
    generator: np.random.Generator, 
    width_bound: float, 
    height_bound: float, 
    n: int
) -> np.ndarray:
    """Generate random robot positions within bounds"""
    x_pos = generator.uniform(low=-width_bound, high=width_bound, size=(n,))
    y_pos = generator.uniform(low=-height_bound, high=height_bound, size=(n,))
    return np.column_stack((x_pos, y_pos))


def validate_config(data: Dict) -> Optional[Response]:
    """Validate simulation configuration"""
    required = [
        "algorithm", "num_of_robots", "robot_speeds", 
        "visibility_radius", "width_bound", "height_bound"
    ]
    
    if any(param not in data for param in required):
        return jsonify({
            "error": "Missing required parameters",
            "required": required
        }), 400
        
    if "initial_positions" in data and len(data["initial_positions"]) != data["num_of_robots"]:
        return jsonify({
            "error": "Initial positions count doesn't match robot count"
        }), 400
        
    return None


@socketio.on("start_simulation")
def handle_simulation_request(data: Dict) -> None:
    """Handle simulation start request with full validation"""
    global simulation_thread, terminate_flag, logger, simulation_id

    # Validate configuration
    if error := validate_config(data):
        emit("config_error", error[0].get_json())
        return

    # Clean up previous simulation
    if simulation_thread and simulation_thread.is_alive():
        logger.info("Simulation interrupted by new request")
        terminate_flag = True
        simulation_thread.join()

    # Initialize simulation
    terminate_flag = False
    seed = data.get("random_seed", np.random.randint(2**32))
    generator = np.random.default_rng(seed=seed)
    
    try:
        initial_positions = (
            data["initial_positions"] 
            if "initial_positions" in data 
            else generate_initial_positions(
                generator, 
                data["width_bound"], 
                data["height_bound"], 
                data["num_of_robots"]
            )
        )
    except Exception as e:
        emit("init_error", {"error": str(e)})
        return

    # Configure logging and scheduler
    logger = setup_logger(simulation_id, data["algorithm"])
    logger.info("Simulation Config:\n%s\n", json.dumps(data, indent=2))

    try:
        scheduler = Scheduler(
            logger=logger,
            seed=seed,
            num_of_robots=data["num_of_robots"],
            initial_positions=initial_positions,
            robot_speeds=data["robot_speeds"],
            rigid_movement=data.get("rigid_movement", True),
            threshold_precision=data.get("threshold_precision", 5),
            sampling_rate=data.get("sampling_rate", 0.2),
            labmda_rate=data.get("labmda_rate", 5),
            algorithm=data["algorithm"],
            visibility_radius=data["visibility_radius"],
            obstructed_visibility=data.get("obstructed_visibility", False),
            fault_prob=min(max(0, data.get("fault_prob", DEFAULT_FAULT_PROB)), 1)
        )
    except Exception as e:
        logger.error(f"Scheduler init failed: {str(e)}")
        emit("init_error", {"error": str(e)})
        return

    def run_simulation() -> None:
        """Main simulation loop with state management"""
        global terminate_flag, simulation_id
        simulation_id += 1

        with app.app_context():
            emit("simulation_start", {"simulation_id": simulation_id})
            
            while not terminate_flag:
                exit_code = scheduler.handle_event()
                
                # Visualization update
                if exit_code == 0 and scheduler.visualization_snapshots:
                    if len(scheduler.visualization_snapshots) > MAX_SNAPSHOTS:
                        scheduler.visualization_snapshots = scheduler.visualization_snapshots[-MAX_SNAPSHOTS:]
                    
                    last_snapshot = scheduler.visualization_snapshots[-1]
                    emit(
                        "simulation_data",
                        {
                            "simulation_id": simulation_id,
                            "snapshot": last_snapshot,
                            "faulty_robots": {
                                r.id: r.fault_type for r in scheduler.robots 
                                if r.fault_type is not None
                            }
                        },
                        broadcast=False
                    )
                
                # Termination handling
                if exit_code < 0:
                    result_data = {
                        "id": simulation_id,
                        "status": "completed",
                        "fault_summary": {
                            "total_faulty": sum(1 for r in scheduler.robots if r.fault_type is not None),
                            "total_robots": data["num_of_robots"],
                            "byzantine": sum(1 for r in scheduler.robots if r.fault_type == "byzantine"),
                            "crash": sum(1 for r in scheduler.robots if r.fault_type == "crash"),
                            "delay": sum(1 for r in scheduler.robots if r.fault_type == "delay")
                        }
                    }

                    if scheduler.robots[0].algorithm == Algorithm.SEC:
                        result_data["sec"] = [r.sec for r in scheduler.robots]
                        result_data["robot_positions"] = [r.pos for r in scheduler.robots]

                    emit("simulation_end", result_data)
                    break

    # Start simulation thread
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()


# SocketIO Events
@socketio.on("connect")
def client_connect() -> None:
    """Handle new client connection"""
    print(f"Client connected: {request.sid}")


@socketio.on("disconnect")
def client_disconnect() -> None:
    """Handle client disconnection"""
    print(f"Client disconnected: {request.sid}")


# Flask Routes
@app.route("/")
def serve_frontend() -> Response:
    """Serve main frontend page"""
    return send_from_directory(app.static_folder, "index.html")


def is_port_in_use(port: int) -> bool:
    """Check if port is available"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def find_available_port(start_port: int = DEFAULT_PORT) -> int:
    """Find next available port"""
    port = start_port
    attempts = 0
    while is_port_in_use(port) and attempts < MAX_PORT_ATTEMPTS:
        port += 1
        attempts += 1
    return port if attempts < MAX_PORT_ATTEMPTS else None


def start_server() -> None:
    """Configure and start the web server"""
    port = find_available_port()
    if port is None:
        print("Error: No available ports found")
        return

    # Open browser after delay
    threading.Timer(1, lambda: webbrowser.open(f"http://127.0.0.1:{port}/")).start()
    
    # Start Flask server
    app.run(
        host="127.0.0.1",
        port=port,
        debug=False,
        use_reloader=False
    )


if __name__ == "__main__":
    start_server()
