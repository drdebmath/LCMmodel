import labels from "./labels.js";
import Queue from "./Queue.js";
import Robot from "./Robot.js";

window.addEventListener("resize", resizeCanvas);

// Elements
let canvas = /** @type {HTMLCanvasElement} */ (document.getElementById("canvas"));
let ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d"));
let time = /**@type {HTMLElement} */ (document.getElementById("time-value"));
let message = /**@type {HTMLElement} */ (document.getElementById("message"));

/** @type {RobotMap}*/
let robots = {};
let snapshotQueue = new Queue();

let paused = false;
let timePerFrameMs = 17;
let lastFrameTime = 0;
let stopAnimation = false;
let currRobotId = 0;
let simulationId = undefined;
let sec = [];
let drawingSimulation = false;

//@ts-ignore
const socket = io(window.location.host);

socket.on("simulation_data", function (data) {
  const _data = JSON.parse(data);
  if (simulationId === _data["simulation_id"]) {
    startDrawingLoop();
    snapshotQueue.enqueue(_data["snapshot"]);
  } else {
    console.log("Received data from mismatched simulation id:");
    console.log(_data);
  }
});

socket.on("simulation_start", function (data) {
  simulationId = data;
  console.log(`Simulation start... ID: ${simulationId}`);
});

socket.on("simulation_end", function () {
  console.log("Simulation complete.");
});

socket.on("smallest_enclosing_circle", function (data) {
  const _data = JSON.parse(data);
  if (simulationId === _data["simulation_id"]) {
    sec = _data["sec"];
  }
});

const schedulerTypes = [labels.Async, labels.Sync];

const algorithmOptions = [labels.Gathering, labels.SEC];

const probabilityDistributions = [labels.Exponential];

const initialPositionsOptions = [labels.Random, labels.UserDefined];

const startSimulation = {
  start_simulation: () => {
    if (
      configOptions.initialization_method === labels.UserDefined &&
      configOptions.initial_positions.length === 0
    ) {
      alert(labels.MissingInitialPositionsAlert);
      return;
    }
    socket.emit("start_simulation", configOptions);
    lastSentConfigOptions = { ...configOptions };
    clearSimulation();
  },
};

const togglePause = {
  pause_simulation: () => {
    paused = true;
  },
  play_simulation: () => {
    paused = false;
  },
};

const clearSimulationObj = {
  clear_simulation: clearSimulation,
};

const configOptions = {
  num_of_robots: 3,
  num_of_faults: 0,
  initialization_method: labels.Random,
  /** @type {Array}*/ initial_positions: [],
  robot_speeds: 1.0,
  robot_size: Robot.ROBOT_SIZE,
  scheduler_type: labels.Async,
  probability_distribution: labels.Exponential,
  visibility_radius: 1500,
  show_visibility: true,
  robot_orientations: null,
  multiplicity_detection: false,
  robot_colors: "#000000",
  obstructed_visibility: false,
  rigid_movement: true,
  threshold_precision: 5,
  sampling_rate: 0.2,
  labmda_rate: 10,
  algorithm: labels.Gathering,
  random_seed: Math.floor(Math.random() * (2 ** 32 - 1)) + 1,
  width_bound: canvas.width / 4,
  height_bound: canvas.height / 4,
};

let lastSentConfigOptions = { ...configOptions };

resizeCanvas();

/**
 * Draws a Robot on the canvas
 * @param {Robot} robot
 */
function drawRobot(robot) {
  ctx.beginPath();

  const color = robot.getColor();
  const radius = Robot.ROBOT_SIZE;

  // Draw circle
  const [x, y] = robot.getCanvasPosition();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.fill();
  ctx.stroke();

  // // Draw node label
  ctx.beginPath();
  ctx.strokeStyle = "#FFF";
  ctx.strokeText(robot.id, x, y);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = `${radius + 1}px Arial`;
  ctx.fill();
  ctx.stroke();

  // Draw multiplicity detection
  if (configOptions.multiplicity_detection) {
    // Draw node label
    ctx.beginPath();
    ctx.strokeStyle = "#000";
    ctx.strokeText("" + robot.multiplicity, x + radius + 1, y - radius - 1);
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.font = `${radius + 1}px Arial`;
    ctx.fill();
    ctx.stroke();
  }

  // Draw visibility radius
  if (configOptions.show_visibility) {
    const vis_radius = drawingSimulation
      ? lastSentConfigOptions.visibility_radius
      : configOptions.visibility_radius;

    ctx.arc(x, y, vis_radius, 0, Math.PI * 2);
    ctx.strokeStyle = "rgb(169 169 169 / 25%)";
    ctx.stroke();
  }
}
/**
 * Draws smallest enclosing circles
 * @param {Circle[]} circles - Smallest Enclosing Circles
 */
function drawSEC(circles) {
  if (circles === undefined || circles.length === 0) {
    return;
  }

  for (const circle of circles) {
    const center_x = circle[0][0] * Robot.ROBOT_X_POS_FACTOR;
    const center_y = circle[0][1] * -1 * Robot.ROBOT_X_POS_FACTOR;
    const radius = circle[1] * Robot.ROBOT_X_POS_FACTOR;
    ctx.strokeStyle = "rgb(169 169 169 / 50%)";

    ctx.beginPath();

    ctx.arc(center_x, center_y, radius, 0, 2 * Math.PI);
    ctx.stroke();
  }
}

const gui = setupOptions(configOptions);

function setupOptions(configOptions) {
  //@ts-ignore
  const gui = new dat.GUI();

  const numRobotsController = gui.add(configOptions, "num_of_robots", 1, 50, 1);
  gui
    .add(configOptions, "initialization_method", initialPositionsOptions)
    .name("Positions")
    .onFinishChange(changeInitializationMethod);
  gui
    .add(configOptions, "num_of_faults", 0, 10, 1)
    .onFinishChange(changeFaults);
  gui.add(configOptions, "rigid_movement");
  gui.add(configOptions, "multiplicity_detection");
  gui.add(configOptions, "obstructed_visibility");
  gui.add(configOptions, "robot_speeds", 1, 20, 0.1);
  gui
    .add(configOptions, "robot_size", Robot.ROBOT_SIZE, 15, 0.5)
    .onFinishChange((size) => Robot.setRobotSize(size));
  gui.add(configOptions, "scheduler_type", schedulerTypes);
  gui.add(configOptions, "probability_distribution", probabilityDistributions);
  gui
    .add(configOptions, "visibility_radius", 50, 1500, 1)
    .onFinishChange(changeVisualizationRadius);
  gui.add(configOptions, "show_visibility");
  gui.add(configOptions, "threshold_precision", 1, 10, 1);
  gui.add(configOptions, "sampling_rate", 0.01, 0.5, 0.01);
  gui.add(configOptions, "labmda_rate");
  gui.add(configOptions, "algorithm", algorithmOptions).name("Algorithm");
  gui.add(configOptions, "random_seed", 1, 2 ** 32 - 1, 1).name("Seed");
  const startSimulationBtn = gui
    .add(startSimulation, "start_simulation")
    .name("Start simulation");
  const pauseBtn = gui
    .add(togglePause, "pause_simulation")
    .name("Pause")
    .onFinishChange(updatePauseText);
  const clearSimulationBtn = gui
    .add(clearSimulationObj, "clear_simulation")
    .name("Clear Simulation");

  startSimulationBtn.domElement.parentElement.parentElement.classList.add("start-btn");

  pauseBtn.domElement.parentElement.parentElement.classList.add("pause-btn");

  clearSimulationBtn.domElement.parentElement.parentElement.classList.add(
    "clear-simulation-btn"
  );

  function updatePauseText() {
    if (paused) {
      pauseBtn.name("Play");
      pauseBtn.property = "play_simulation";
    } else {
      pauseBtn.name("Pause");
      pauseBtn.property = "pause_simulation";
    }
  }


  function changeInitializationMethod() {
    const numRobotsControllerElement = numRobotsController.domElement;
    if (configOptions.initialization_method === labels.Random) {
      numRobotsControllerElement.parentElement.parentElement.style.display = "list-item";
      numRobotsController.setValue(3);

      canvas.removeEventListener("click", handleCanvasClick);

      clearSimulation();
    } else {
      numRobotsControllerElement.parentElement.parentElement.style.display = "none";
      numRobotsController.setValue(0);
      message.style.display = "block";

      canvas.addEventListener("click", handleCanvasClick);

      clearSimulation();
    }
  }

  function changeVisualizationRadius() {
    if (configOptions.initial_positions.length != 0) {
      clearCanvas();
      for (const id in robots) {
        drawRobot(robots[id]);
      }
    }
  }

  function changeFaults() {
    if (configOptions.num_of_faults > configOptions.num_of_robots) {
      configOptions.num_of_faults = configOptions.num_of_robots;
    }
    if (configOptions.num_of_faults < 0) {
      configOptions.num_of_faults = 0;
    }
    else{
      configOptions.num_of_faults = Math.floor(configOptions.num_of_faults);
    }
    clearSimulation();
  }

  return { gui, updatePauseText };
}

function startDrawingLoop() {
  stopAnimation = false;
  drawingSimulation = true;

  // requestAnimationFrame initiates the animation loop
  requestAnimationFrame(drawLoop); // Start the loop
}

function stopDrawingLoop() {
  stopAnimation = true;
  drawingSimulation = false;
}

function drawLoop(currentTime) {
  if (stopAnimation) {
    return;
  }

  if (simulationId === undefined) {
    return;
  }
  // Calculate the time since the last frame
  const deltaTime = currentTime - lastFrameTime;

  // Check if enough time has passed to render a new frame
  if (deltaTime >= timePerFrameMs && !paused) {
    const snapshot = snapshotQueue.dequeue();
    if (snapshot) {
      clearCanvas();
      drawSnapshot(snapshot);
      lastFrameTime = currentTime; // Reset lastFrameTime for the next frame
    } else {
      stopDrawingLoop();
      drawSEC(sec);

      return;
    }
  }

  // Request the next frame
  requestAnimationFrame(drawLoop);
}

function clearCanvas() {
  ctx.clearRect(-canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(-canvas / 2, -canvas.height / 2, canvas.width, canvas.height);
}

function getRandomColor() {
  const r = Math.floor(50 + Math.random() * 256);
  const g = Math.floor(50 + Math.random() * 256);
  const b = Math.floor(50 + Math.random() * 256);

  return `rgb(${r}, ${g}, ${b})`;
}

function resizeCanvas() {
  // This function resets the canvas
  console.log("Resized Canvas");

  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;

  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Translate the coordinate system to be in the center
  ctx.translate(canvas.width / 2, canvas.height / 2);
  configOptions.width_bound = canvas.width / 2;
  configOptions.height_bound = canvas.height / 2;
}

/**
 * @param {Snapshot} snapshot
 * */
function drawSnapshot(snapshot) {
  let time = snapshot[0];
  updateTimeElement(time);

  let robotsHistory = snapshot[1];

  for (let id in robotsHistory) {
    let [x, y] = robotsHistory[id][0];
    const multiplicity = robotsHistory[id][4];
    const state = robotsHistory[id][1];

    if (robots[id] === undefined) {
      robots[id] = new Robot(x, y, id, "black", 1, multiplicity);
    }

    robots[id].setPosition(x, y);
    robots[id].setState(state);
    robots[id].multiplicity = multiplicity;
    drawRobot(robots[id]);
  }
}

function updateTimeElement(t) {
  time.innerText = t;
}

/**
 * @param {MouseEvent} e
 */
function handleCanvasClick(e) {
  if (time.innerText !== "") {
    clearSimulation();
  }

  console.log(e);
  const x = e.offsetX;
  const y = e.offsetY;

  const [canvasX, canvasY] = translateToCanvas(canvas, x, y);

  const robot = new Robot(
    canvasX,
    canvasY,
    `${currRobotId++}`,
    configOptions.robot_colors,
    configOptions.robot_speeds,
    1,
    true
  );

  robots[currRobotId - 1] = robot;

  drawRobot(robot);

  configOptions.initial_positions.push(robot.getPosition());
  message.style.display = "none";
}

function clearSimulation() {
  clearCanvas();
  updateTimeElement("");

  simulationId = undefined;
  snapshotQueue = new Queue();
  robots = {};
  lastFrameTime = 0;
  currRobotId = 0;
  configOptions.initial_positions = [];
  paused = false;
  gui.updatePauseText();
  sec = [];
  drawingSimulation = false;
}

/**
 * Converts mouse coordinates to coordinates on a canvas with origin at the center of screen
 * @param {HTMLCanvasElement} canvas - Canvas element
 * @param {number} x
 * @param {number} y
 */
function translateToCanvas(canvas, x, y) {
  return [x - canvas.width / 2, y - canvas.height / 2];
}
