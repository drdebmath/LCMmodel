const SIMULATION_SCALE_FACTOR = 100;
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
    // Collect fault configurations
    configOptions.robot_faults = [];
    for (let i = 0; i < configOptions.num_of_robots; i++) {
      const faultType = document.getElementById(`fault-type-${i}`)?.value || 'NONE';
      const faultProb = parseFloat(document.getElementById(`fault-prob-${i}`)?.value) || 0.3;
      
      configOptions.robot_faults.push({
        type: faultType,
        probability: faultProb
      });
    }
    
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
  robot_faults: [],
};

let lastSentConfigOptions = { ...configOptions };

// Initialize canvas and event listeners
resizeCanvas();
setupInitialEventListeners();

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

  // Draw node label
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

  // Enhanced fault display
  if (robot.fault_type && robot.fault_type !== 'NONE') {
    ctx.font = '10px Arial';
    ctx.fillStyle = robot.fault_status === 'TRIGGERED' ? 'red' : 'orange';
    ctx.textAlign = 'center';
    
    const statusText = robot.fault_status ? ` (${robot.fault_status})` : '';
    ctx.fillText(
      `${robot.fault_type}${statusText}`, 
      x, 
      y + Robot.ROBOT_SIZE + 12
    );
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

function setupInitialEventListeners() {
  // Always bind click handler, but filter in handler
  canvas.addEventListener("click", handleCanvasClick);
  updateInitializationMessage();
}

function updateInitializationMessage() {
  message.style.display = 
    configOptions.initialization_method === labels.UserDefined 
      ? "block" 
      : "none";
}

function createFaultSelectionUI() {
  const faultContainer = document.createElement('div');
  faultContainer.id = 'fault-container';
  faultContainer.style.margin = '20px';
  faultContainer.style.padding = '10px';
  faultContainer.style.border = '1px solid #ccc';
  
  const title = document.createElement('h3');
  title.textContent = 'Robot Fault Configuration';
  faultContainer.appendChild(title);

  const description = document.createElement('p');
  description.textContent = 'Select fault type for each robot. Faults will activate during simulation.';
  description.style.fontSize = '0.9em';
  description.style.color = '#666';
  faultContainer.appendChild(description);

  const table = document.createElement('table');
  table.style.width = '100%';
  table.style.marginTop = '10px';
  table.style.borderCollapse = 'collapse';

  const thead = document.createElement('thead');
  const headerRow = document.createElement('tr');
  ['Robot ID', 'Fault Type', 'Activation Probability'].forEach(text => {
    const th = document.createElement('th');
    th.textContent = text;
    th.style.textAlign = 'left';
    th.style.padding = '8px';
    th.style.borderBottom = '1px solid #ddd';
    headerRow.appendChild(th);
  });
  thead.appendChild(headerRow);
  table.appendChild(thead);

  const tbody = document.createElement('tbody');

  const faultOptions = [
    { value: 'NONE', label: 'No Fault' },
    { value: 'CRASH', label: 'Crash (Instant Termination)' },
    { value: 'DELAY', label: 'Delay (50% Speed)' },
    { value: 'WRONG_COMPUTE', label: 'Wrong Computation' },
    { value: 'VISIBILITY', label: 'Partial Visibility' },
    { value: 'MOVEMENT', label: 'Inverted Movement' }
  ];

  for (let i = 0; i < configOptions.num_of_robots; i++) {
    const tr = document.createElement('tr');
    
    const idCell = document.createElement('td');
    idCell.textContent = `Robot ${i}`;
    idCell.style.padding = '8px';
    idCell.style.borderBottom = '1px solid #eee';
    tr.appendChild(idCell);

    const typeCell = document.createElement('td');
    const select = document.createElement('select');
    select.id = `fault-type-${i}`;
    select.name = `fault-type-${i}`;
    select.style.width = '100%';
    select.style.padding = '4px';
    
    faultOptions.forEach(option => {
      const optElement = document.createElement('option');
      optElement.value = option.value;
      optElement.textContent = option.label;
      select.appendChild(optElement);
    });
    
    typeCell.appendChild(select);
    typeCell.style.padding = '8px';
    typeCell.style.borderBottom = '1px solid #eee';
    tr.appendChild(typeCell);

    const probCell = document.createElement('td');
    const probInput = document.createElement('input');
    probInput.type = 'number';
    probInput.id = `fault-prob-${i}`;
    probInput.name = `fault-prob-${i}`;
    probInput.min = '0';
    probInput.max = '1';
    probInput.step = '0.1';
    probInput.value = '0.3';
    probInput.style.width = '60px';
    probInput.style.padding = '4px';
    probCell.appendChild(probInput);
    probCell.style.padding = '8px';
    probCell.style.borderBottom = '1px solid #eee';
    tr.appendChild(probCell);

    tbody.appendChild(tr);
  }

  table.appendChild(tbody);
  faultContainer.appendChild(table);

  const guiContainer = document.querySelector('.dg.main.a');
  if (guiContainer) {
    guiContainer.parentNode.insertBefore(faultContainer, guiContainer.nextSibling);
  } else {
    document.body.appendChild(faultContainer);
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
      updateInitializationMessage();
      if (document.getElementById('fault-container')) {
        document.getElementById('fault-container').remove();
      }
      createFaultSelectionUI();
    } else {
      numRobotsControllerElement.parentElement.parentElement.style.display = "none";
      numRobotsController.setValue(0);
      updateInitializationMessage();
      if (document.getElementById('fault-container')) {
        document.getElementById('fault-container').remove();
      }
    }
    clearSimulation();
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
  if (stopAnimation) {
    return;
  }

  if (simulationId === undefined) {
    return;
  }

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

function resizeCanvas() {
  console.log("Resized Canvas");
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;

  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Reset transform and set new center
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.translate(canvas.width / 2, canvas.height / 2);
  
  configOptions.width_bound = canvas.width / 2;
  configOptions.height_bound = canvas.height / 2;
}

/**
 * @param {Snapshot} snapshot
 */
function drawSnapshot(snapshot) {
  let time = snapshot[0];
  updateTimeElement(time);

  let robotsHistory = snapshot[1];

  for (let id in robotsHistory) {
    let [x, y] = robotsHistory[id][0];
    const multiplicity = robotsHistory[id][4];
    const state = robotsHistory[id][1];
    const fault_type = robotsHistory[id][5] || 'NONE';
    const fault_status = robotsHistory[id][6] || 'INACTIVE';

    if (robots[id] === undefined) {
      robots[id] = new Robot(x, y, id, "black", 1, multiplicity);
    }

    robots[id].setPosition(x, y);
    robots[id].setState(state);
    robots[id].multiplicity = multiplicity;
    robots[id].fault_type = fault_type;
    robots[id].fault_status = fault_status;
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
  // Only process in UserDefined mode
  if (configOptions.initialization_method !== labels.UserDefined) return;

  if (time.innerText !== "") {
    clearSimulation();
  }

  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left - canvas.width/2;
  const y = e.clientY - rect.top - canvas.height/2;
  
  const simX = x / Robot.ROBOT_X_POS_FACTOR;
  const simY = -y / Robot.ROBOT_X_POS_FACTOR;

  console.log(`Clicked at screen: (${e.clientX}, ${e.clientY})`);
  console.log(`Canvas coordinates: (${x}, ${y})`);
  console.log(`Simulation coordinates: (${simX}, ${simY})`);

  // Visual feedback
  ctx.fillStyle = 'rgba(0, 200, 0, 0.7)';
  ctx.beginPath();
  ctx.arc(x, y, 15, 0, Math.PI*2);
  ctx.fill();

  // Add robot
  const robot = new Robot(
    simX, 
    simY,
    `${currRobotId++}`,
    configOptions.robot_colors,
    configOptions.robot_speeds,
    1,
    true
  );
  
  robots[currRobotId-1] = robot;
  drawRobot(robot);
  configOptions.initial_positions.push([simX, simY]);
  updateInitializationMessage();
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
  configOptions.robot_faults = [];
  paused = false;
  if (gui && gui.updatePauseText) {
    gui.updatePauseText();
  }
  sec = [];
  drawingSimulation = false;
  
  const faultContainer = document.getElementById('fault-container');
  if (faultContainer) {
    faultContainer.remove();
  }
}

// Create fault UI when page loads
window.addEventListener('load', () => {
  if (configOptions.initialization_method === labels.Random) {
    createFaultSelectionUI();
  }
});
