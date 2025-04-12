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


def get_log_name():
    date = datetime.now()
    milliseconds = date.microsecond // 1000

    return f"{date.year}-{date.month}-{date.day}-{date.hour}-{date.minute}-{date.second}-{milliseconds}.txt"


def setup_logger(simulation_id, algo_name):
    logger = logging.getLogger(f"app_{simulation_id}")
    logger.setLevel(logging.INFO)

    # Add a new file handler
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


# Disable Flask's default logging to the root logger
log = logging.getLogger(
    "werkzeug"
)  # 'werkzeug' is the logger used by Flask for requests
log.setLevel(logging.ERROR)  # Set Flask's logging to a different level or disable it

app = Flask(__name__, static_folder="static")
socketio = SocketIO(app)

simulation_thread = None
terminate_flag = False
simulation_id = 0
logger = None


# WebSocket event handler for the simulation
@socketio.on("start_simulation")
def handle_simulation_request(data):
    global simulation_thread, terminate_flag, logger

    # Terminate existing simulation thread
    if simulation_thread and simulation_thread.is_alive():
        logger.info("Simulation Interrupted... (A new simulation was requested)")
        terminate_flag = True
        simulation_thread.join()

    # Reset the termination flag and start a new simulation thread
    terminate_flag = False

    seed = data["random_seed"]
    seed = 2708382154
    generator = np.random.default_rng(seed=seed)
    num_robots = data["num_of_robots"]
    initial_positions: list = data["initial_positions"]

    if len(initial_positions) != 0:
        # User defined
        initial_positions = data["initial_positions"]
        num_robots = len(initial_positions)
    else:
        # Random
        initial_positions = generate_initial_positions(
            generator, data["width_bound"], data["height_bound"], num_robots
        )

    logger = setup_logger(simulation_id, data["algorithm"])
    logger.info("Config:\n\n%s\n", json.dumps(data, indent=2))

    scheduler = Scheduler(
        logger=logger,
        seed=seed,
        num_of_robots=num_robots,
        initial_positions=initial_positions,
        robot_speeds=data["robot_speeds"],
        rigid_movement=data["rigid_movement"],
        threshold_precision=data["threshold_precision"],
        sampling_rate=data["sampling_rate"],
        labmda_rate=data["labmda_rate"],
        algorithm=data["algorithm"],
        visibility_radius=data["visibility_radius"],
        num_of_faults=data["num_of_faults"],
    )

    def run_simulation():
        global terminate_flag, simulation_id

        simulation_id += 1

        with app.app_context():
            socketio.emit("simulation_start", simulation_id)
            while terminate_flag != True:
                exit_code = scheduler.handle_event()
                if exit_code == 0:
                    snapshots = scheduler.visualization_snapshots
                    if len(snapshots) > 0:
                        socketio.emit(
                            "simulation_data",
                            json.dumps(
                                {
                                    "simulation_id": simulation_id,
                                    "snapshot": scheduler.visualization_snapshots[-1],
                                }
                            ),
                        )

                if exit_code < 0:
                    # Signal the end of the simulation
                    if scheduler.robots[0].algorithm == Algorithm.SEC:
                        socketio.emit(
                            "smallest_enclosing_circle",
                            json.dumps(
                                {
                                    "simulation_id": simulation_id,
                                    "sec": [
                                        scheduler.robots[i].sec
                                        for i in range(num_robots)
                                    ],
                                }
                            ),
                        )
                    socketio.emit("simulation_end", "END")
                    break

    # Start a thread for the simulation to not block websocket
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()


@socketio.on("connect")
def client_connect():
    print("Client connected")


@socketio.on("disconnect")
def client_disconnect():
    print("Client disconnected")


@app.route("/")
def serve_frontend():
    return send_from_directory(app.static_folder, "index.html")


def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("0.0.0.0", port)) == 0


def open_browser(host, port):
    # If host is 0.0.0.0, use localhost for browser opening
    browser_host = "localhost" if host == "0.0.0.0" else host
    webbrowser.open(f"http://{browser_host}:{port}/")


host = "0.0.0.0"  # Change from 127.0.0.1 to 0.0.0.0 to allow external access
port = 8080
while is_port_in_use(port):
    port += 1

# Open the browser after a delay to let the server start
threading.Timer(1, open_browser, args=(host, port)).start()
app.run(host=host, port=port, debug=True, use_reloader=False)
