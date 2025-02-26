import labels from "./labels.js";
import Queue from "./Queue.js";
import Robot from "./Robot.js";

// General UI
let canvas = /** @type {HTMLCanvasElement} */ (document.getElementById("canvas"));
let ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d"));
let time = /** @type {HTMLElement} */ (document.getElementById("time-value"));
let message = /** @type {HTMLElement} */ (document.getElementById("message"));
let controls_ui = /** @type {HTMLElement} */ (document.getElementById('controls-ui-container'));

window.addEventListener("load", resizeCanvas);
window.addEventListener("resize", resizeCanvas);

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
const algorithmOptions = [labels.Gathering, labels.SEC, labels.Custom];
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
  initialization_method: labels.Random,
  initial_positions: [],
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
  custom_alg: "",
  custom_term_code: "",
  random_seed: Math.floor(Math.random() * (2 ** 32 - 1)) + 1,
  width_bound: canvas.width / 4,
  height_bound: canvas.height / 4,
};

let lastSentConfigOptions = { ...configOptions };

function drawRobot(robot) {
  ctx.beginPath();
  const color = robot.getColor();
  const radius = Robot.ROBOT_SIZE;
  const [x, y] = robot.getCanvasPosition();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.strokeStyle = "#FFF";
  ctx.strokeText(robot.id, x, y);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = `${radius + 1}px Arial`;
  ctx.fill();
  ctx.stroke();

  if (configOptions.multiplicity_detection) {
    ctx.beginPath();
    ctx.strokeStyle = "#000";
    ctx.strokeText("" + robot.multiplicity, x + radius + 1, y - radius - 1);
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.font = `${radius + 1}px Arial`;
    ctx.fill();
    ctx.stroke();
  }

  if (configOptions.show_visibility) {
    const vis_radius = drawingSimulation
      ? lastSentConfigOptions.visibility_radius
      : configOptions.visibility_radius;
    ctx.arc(x, y, vis_radius, 0, Math.PI * 2);
    ctx.strokeStyle = "rgb(169 169 169 / 25%)";
    ctx.stroke();
  }
}

function drawSEC(circles) {
  if (!circles || circles.length === 0) return;
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
  const gui = new dat.GUI({ autoPlace: false });
  controls_ui.append(gui.domElement);
  const numRobotsController = gui.add(configOptions, "num_of_robots", 1, 50, 1);
  gui
    .add(configOptions, "initialization_method", initialPositionsOptions)
    .name("Positions")
    .onFinishChange(changeInitializationMethod);
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
  const startSimulationBtn = gui.add(startSimulation, "start_simulation").name("Start simulation");
  const pauseBtn = gui.add(togglePause, "pause_simulation").name("Pause").onFinishChange(updatePauseText);
  const clearSimulationBtn = gui.add(clearSimulationObj, "clear_simulation").name("Clear Simulation");

  startSimulationBtn.domElement.parentElement.parentElement.classList.add("start-btn");
  pauseBtn.domElement.parentElement.parentElement.classList.add("pause-btn");
  clearSimulationBtn.domElement.parentElement.parentElement.classList.add("clear-simulation-btn");

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

  return { gui, updatePauseText };
}

function startDrawingLoop() {
  stopAnimation = false;
  drawingSimulation = true;
  requestAnimationFrame(drawLoop);
}

function stopDrawingLoop() {
  stopAnimation = true;
  drawingSimulation = false;
}

function drawLoop(currentTime) {
  if (stopAnimation) return;
  if (simulationId === undefined) return;
  const deltaTime = currentTime - lastFrameTime;
  if (deltaTime >= timePerFrameMs && !paused) {
    const snapshot = snapshotQueue.dequeue();
    if (snapshot) {
      clearCanvas();
      drawSnapshot(snapshot);
      lastFrameTime = currentTime;
    } else {
      stopDrawingLoop();
      drawSEC(sec);
      return;
    }
  }
  requestAnimationFrame(drawLoop);
}

function clearCanvas() {
  ctx.clearRect(-canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(-canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
}

function getRandomColor() {
  const r = Math.floor(50 + Math.random() * 256);
  const g = Math.floor(50 + Math.random() * 256);
  const b = Math.floor(50 + Math.random() * 256);
  return `rgb(${r}, ${g}, ${b})`;
}

function resizeCanvas() {
  console.log("Resized Canvas");
  const canvas_parent_rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = canvas_parent_rect.width;
  canvas.height = canvas_parent_rect.height;
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.translate(canvas.width / 2, canvas.height / 2);
  configOptions.width_bound = canvas.width / 2;
  configOptions.height_bound = canvas.height / 2;
}

function drawSnapshot(snapshot) {
  let timeVal = snapshot[0];
  updateTimeElement(timeVal);
  let robotsHistory = snapshot[1];
  for (let id in robotsHistory) {
    let [x, y] = robotsHistory[id][0];
    const state = robotsHistory[id][1];
    const multiplicity = robotsHistory[id][4];
    const light = robotsHistory[id][5];  // new field for light
    if (robots[id] === undefined) {
      robots[id] = new Robot(x, y, id, light, 1, multiplicity);
    } else {
      robots[id].setColor(light);
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

function handleCanvasClick(e) {
  if (time.innerText !== "") {
    clearSimulation();
  }
  console.log(e);
  const x = e.offsetX;
  const y = e.offsetY;
  const [canvasX, canvasY] = translateToCanvas(canvas, x, y);
  const robot = new Robot(canvasX, canvasY, `${currRobotId++}`, configOptions.robot_colors, configOptions.robot_speeds, 1, true);
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

function translateToCanvas(canvas, x, y) {
  return [x - canvas.width / 2, y - canvas.height / 2];
}

export default configOptions;