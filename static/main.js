import labels from "./labels.js";
import Queue from "./Queue.js";
import Robot from "./Robot.js";

// General UI
let canvas = /** @type {HTMLCanvasElement} */ (document.getElementById("canvas"));
let ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d", { alpha: false }));
let time = /** @type {HTMLElement} */ (document.getElementById("time-value"));
let message = /** @type {HTMLElement} */ (document.getElementById("message"));
let controls_ui = /** @type {HTMLElement} */ (document.getElementById('controls-ui-container'));

window.addEventListener("load", resizeCanvas);
window.addEventListener("resize", resizeCanvas);

// Performance optimizations
const RENDER_THROTTLE_MS = 16; // Cap at ~60fps
let isFirstRender = true;
let lastRenderTime = 0;
let robots = {};
let snapshotQueue = new Queue();
let dirtyRegions = new Set(); // Track regions that need redrawing
let robotPrevPositions = {}; // Track previous positions for partial rendering

let paused = false;
let timePerFrameMs = 17;
let lastFrameTime = 0;
let stopAnimation = false;
let currRobotId = 0;
let simulationId = undefined;
let sec = [];
let drawingSimulation = false;

// Viewport adjustment variables
let viewScale = 1.0;
let viewCenterX = 0;
let viewCenterY = 0;
let needsViewportUpdate = false;
let pendingViewportUpdate = false;

// Frame rate tracking
let frameCount = 0;
let lastFpsTime = 0;
let fps = 0;
let showFps = false; // Set to true to enable FPS counter

// Web Worker for data processing
let dataWorker = null;

// More robust socket initialization
let socket;

try {
  console.log("Initializing socket.io...");
  console.log("Window location:", window.location);
  console.log("Window location host:", window.location.host);
  
  // First try to connect to the host
  socket = io(window.location.host, {
    reconnectionAttempts: 5,
    timeout: 10000,
    transports: ['websocket', 'polling'],
    upgrade: true,
    path: '/socket.io',
    forceNew: true,
    autoConnect: true
  });
  
  console.log("Socket.io initialized with host:", window.location.host);
  
  // Add connection events
  socket.on('connect', () => {
    console.log('Socket.io connected successfully');
    console.log('Socket ID:', socket.id);
  });
  
  socket.on('connect_error', (error) => {
    console.error('Socket.io connection error:', error);
    // Try to reconnect with a different path
    if (!socket.connected && !socket._reconnectionAttempted) {
      console.log('Attempting alternative connection...');
      socket._reconnectionAttempted = true;
      // Try with explicit path
      socket = io(window.location.host, {
        path: '/socket.io',
        reconnectionAttempts: 3,
        transports: ['websocket', 'polling'],
        upgrade: true,
        forceNew: true,
        autoConnect: true
      });
    }
  });

  socket.on('disconnect', (reason) => {
    console.log('Socket disconnected:', reason);
  });

  socket.on('reconnect', (attemptNumber) => {
    console.log('Socket reconnected after', attemptNumber, 'attempts');
  });

  socket.on('reconnect_error', (error) => {
    console.error('Socket reconnection error:', error);
  });

  socket.on('reconnect_failed', () => {
    console.error('Socket reconnection failed');
  });

  socket.on('error', (error) => {
    console.error('Socket error:', error);
  });
} catch (e) {
  console.error("Error initializing socket.io:", e);
  // Fallback to a basic socket
  socket = {
    on: function() { console.warn("Socket not available"); },
    emit: function() { console.warn("Socket not available"); },
    connected: false
  };
}

// Make socket globally available for other components
window.socket = socket;

function checkSocketConnection() {
  if (!socket.connected) {
    console.log("Socket not connected. Attempting to reconnect...");
    socket.connect();
  }
}

// Add connection event handlers
socket.on('connect', function() {
  console.log("Socket connected successfully!");
  console.log("Socket ID:", socket.id);
});

socket.on('connect_error', function(error) {
  console.error("Socket connection error:", error);
});

socket.on("simulation_data", function (data) {
  console.log("Received simulation data");
  try {
    const _data = JSON.parse(data);
    console.log("Parsed data:", _data);
    if (simulationId === _data["simulation_id"]) {
      console.log("Processing snapshot:", _data["snapshot"]);
      startDrawingLoop();
      snapshotQueue.enqueue(_data["snapshot"]);
    } else {
      console.log("Received data from mismatched simulation id:");
      console.log("Expected:", simulationId);
      console.log("Received:", _data["simulation_id"]);
      console.log("Full data:", _data);
    }
  } catch (e) {
    console.error("Error processing simulation data:", e);
  }
});

socket.on("simulation_data_batch", function (data) {
  const _data = JSON.parse(data);
  if (!_data.batch || !Array.isArray(_data.batch)) {
    console.warn("Received invalid batch data");
    return;
  }
  
  console.log(`Received batch of ${_data.batch.length} snapshots`);
  
  // Process batch with worker if available
  if (dataWorker) {
    dataWorker.postMessage({
      type: 'process_batch',
      data: _data.batch
    });
  } else {
    // Fallback to synchronous processing
    if (_data.batch.length > 0) {
      startDrawingLoop();
      
      // Check if the batch matches our simulation ID
      const firstItem = _data.batch[0];
      if (simulationId !== firstItem.simulation_id) {
        console.log("Received batch from mismatched simulation id");
        return;
      }
      
      // Enqueue all snapshots
      for (const item of _data.batch) {
        snapshotQueue.enqueue(item.snapshot);
      }
    }
  }
  
  // Acknowledge batch receipt
  socket.emit("batch_ack", {
    count: _data.batch.length,
    simulation_id: simulationId
  });
});

socket.on("simulation_start", function (data) {
  console.log("Received simulation start event");
  console.log("Data:", data);
  simulationId = data;
  console.log(`Simulation start... ID: ${simulationId}`);
  // Reset optimization tracking data
  dirtyRegions.clear();
  robotPrevPositions = {};
  
  // Clear worker cache
  if (dataWorker) {
    dataWorker.postMessage({ type: 'clear_cache' });
  }
});

socket.on("simulation_end", function () {
  console.log("Simulation complete.");
  // Draw final SEC circles
  if (sec.length > 0) {
    requestAnimationFrame(() => {
      drawSEC(sec);
    });
  }
});

socket.on("simulation_stats", function (data) {
  const _data = JSON.parse(data);
  console.log("Simulation statistics:", _data);
  
  // Process extended stats with worker if available
  if (dataWorker) {
    dataWorker.postMessage({
      type: 'calculate_stats',
      data: _data
    });
  }
});

socket.on("smallest_enclosing_circle", function (data) {
  const _data = JSON.parse(data);
  if (simulationId === _data["simulation_id"]) {
    sec = _data["sec"];
  }
});

// Configuration options
const schedulerTypes = [labels.Async, labels.Sync];
const algorithmOptions = [labels.Gathering, labels.SEC, labels.Custom, labels.ColorBased, labels.TwoRobot, labels.MutualVisibility];
const probabilityDistributions = [labels.Exponential];
const initialPositionsOptions = [labels.Random, labels.UserDefined];

const startSimulation = {
  start_simulation: function() {
    console.log("Starting simulation with config:", configOptions);
    
    // Clear any existing simulation
    clearSimulation();
    
    // Generate a new simulation ID
    simulationId = Math.random().toString(36).substring(7);
    console.log("Generated simulation ID:", simulationId);
    
    // Send simulation request to server
    socket.emit("start_simulation", {
      ...configOptions,
      simulation_id: simulationId
    });
    
    // Start the drawing loop
    startDrawingLoop();
  }
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

  sampling_rate: 1.0,          // Increased from 0.2 to 1.0 for fewer snapshots
  lambda_rate: 10,
  algorithm: labels.Gathering,
  custom_alg: "",
  custom_term_code: "",
  random_seed: Math.floor(Math.random() * (2 ** 32 - 1)) + 1,
  width_bound: canvas.width / 4,
  height_bound: canvas.height / 4,
  
  // New property for TwoRobot scenario selection
  twoRobotScenario: "One Black, One White", // Default scenario
};

let lastSentConfigOptions = { ...configOptions };
// Update the drawRobot function in main.js to enhance white robot visibility

function drawRobot(robot) {
  // Skip drawing robots that are outside the visible canvas area
  const [x, y] = robot.getCanvasPosition();
  const robotRadius = Robot.ROBOT_SIZE;
  const visibilityRadius = configOptions.show_visibility ? configOptions.visibility_radius : 0;
  const maxRadius = Math.max(robotRadius * 2, 30) + (configOptions.show_visibility ? configOptions.visibility_radius : 0);
  
  // Check if robot is outside visible area with a margin
  if (x + maxRadius < 0 || x - maxRadius > canvas.width || 
      y + maxRadius < 0 || y - maxRadius > canvas.height) {
    return; // Skip drawing this robot
  }
  
  // Create a dirty region for this robot
  const dirtySize = Math.max(robotRadius * 2, 30) + (configOptions.show_visibility ? configOptions.visibility_radius : 0);
  addDirtyRegion(x, y, dirtySize);

  // Draw robot body
  ctx.beginPath();
  ctx.arc(x, y, robotRadius, 0, Math.PI * 2);
  ctx.fillStyle = robot.getColor();
  
  // Enhanced visibility for different colors
  const robotColor = robot.getColor().toUpperCase();
  if (robotColor === "#FFFFFF" || robotColor === "#FF0000" || robotColor === "#00FF00") {
    // Use a darker and thicker border for better contrast
    ctx.strokeStyle = "#000000";
    ctx.lineWidth = 3;
    ctx.stroke();
    
    // Add a second circle with different color for even more visibility
    ctx.beginPath();
    ctx.arc(x, y, robotRadius - 3, 0, Math.PI * 2);
    ctx.strokeStyle = "#444444";
    ctx.lineWidth = 1;
    ctx.stroke();
  } else {
    ctx.strokeStyle = "#000000";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  
  ctx.fill();
  
  // Draw robot ID with better contrast
  ctx.beginPath();
  // For light colored robots, use black text
  ctx.fillStyle = (robotColor === "#FFFFFF" || robotColor === "#00FF00") ? "#000000" : "#FFFFFF";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = `${robotRadius}px Arial`;
  ctx.fillText(robot.id, x, y);
  
  // For light colored robots, draw with a slight offset for better visibility
  if (robotColor === "#FFFFFF" || robotColor === "#00FF00") {
    ctx.strokeStyle = "#000000";
    ctx.lineWidth = 0.5;
    ctx.strokeText(robot.id, x, y);
  }

  // Draw multiplicity if enabled
  if (configOptions.multiplicity_detection) {
    ctx.beginPath();
    ctx.fillStyle = "#000000";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.font = `${robotRadius}px Arial`;
    ctx.fillText("" + robot.multiplicity, x + robotRadius + 1, y - robotRadius - 1);
  }

  // Draw visibility circle if enabled
  if (configOptions.show_visibility) {
    const vis_radius = drawingSimulation
      ? lastSentConfigOptions.visibility_radius
      : configOptions.visibility_radius;
    ctx.beginPath();  
    ctx.arc(x, y, vis_radius, 0, Math.PI * 2);
    ctx.strokeStyle = "rgb(169 169 169 / 25%)";
    ctx.stroke();
  }
  
  // Add color label for better visibility of state
  ctx.beginPath();
  ctx.fillStyle = "#000000";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = `${robotRadius * 0.8}px Arial`;
  const colorLabel = robotColor === "#FFFFFF" ? "W" : 
                    robotColor === "#000000" ? "B" :
                    robotColor === "#FF0000" ? "R" :
                    robotColor === "#00FF00" ? "G" : "?";
  ctx.fillText(colorLabel, x, y + robotRadius * 1.8);
}

// Add a dirty region that needs to be redrawn
function addDirtyRegion(x, y, size) {
  dirtyRegions.add({x: x, y: y, size: size});
}

// Clear only the dirty regions instead of the whole canvas
function clearDirtyRegions() {
  for (const region of dirtyRegions) {
    const size = region.size;
    ctx.clearRect(region.x - size, region.y - size, size * 2, size * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(region.x - size, region.y - size, size * 2, size * 2);
  }
  dirtyRegions.clear();
}

function drawSEC(circles) {
  if (!circles || circles.length === 0) return;
  
  ctx.save();
  for (const circle of circles) {
    const center_x = circle[0][0] * Robot.ROBOT_X_POS_FACTOR;
    const center_y = circle[0][1] * Robot.ROBOT_Y_POS_FACTOR; // Fixed to match coordinate system
    const radius = circle[1] * Robot.ROBOT_X_POS_FACTOR;
    
    // Add as a dirty region to ensure it gets drawn properly
    addDirtyRegion(center_x, center_y, radius);
    
    ctx.strokeStyle = "rgb(169 169 169 / 50%)";
    ctx.beginPath();
    ctx.arc(center_x, center_y, radius, 0, 2 * Math.PI);
    ctx.stroke();
  }
  ctx.restore();
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
    
  // Add the Two-Robot Scenario dropdown
  const twoRobotScenarioController = gui
    .add(configOptions, "twoRobotScenario", [
      "One Black, One White",
      "Two Black",
      "Two White"
    ])
    .name("TwoRobot Scenario")
    .onChange(function(value) {
      // Update the seed to a special value to force the scenario
      // These values must match exactly with the backend constants
      if (value === "Two Black") {
        configOptions.random_seed = 1000000001;  // TWO_BLACK_SEED
      } else if (value === "Two White") {
        configOptions.random_seed = 2000000001;  // TWO_WHITE_SEED
      } else {
        configOptions.random_seed = 3000000001;  // ONE_EACH_SEED
      }
      console.log(`Setting seed to ${configOptions.random_seed} for scenario: ${value}`);
    });
  
  // Make the scenario selector only visible when TwoRobot algorithm is selected
  const algorithmController = gui.add(configOptions, "algorithm", algorithmOptions).name("Algorithm");
  algorithmController.onChange(function(value) {
    const isTwoRobot = value === "TwoRobot";
    twoRobotScenarioController.domElement.parentElement.parentElement.style.display = 
      isTwoRobot ? "block" : "none";
  });
  
  // Hide the scenario selector initially if not TwoRobot
  if (configOptions.algorithm !== "TwoRobot") {
    twoRobotScenarioController.domElement.parentElement.parentElement.style.display = "none";
  }
    
  gui.add(configOptions, "rigid_movement");
  gui.add(configOptions, "multiplicity_detection");
  gui.add(configOptions, "obstructed_visibility").name("Obstructed Visibility");
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
  gui.add(configOptions, "lambda_rate");
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
      // Force redraw with new visibility settings
      needsViewportUpdate = true;
      for (const id in robots) {
        const [x, y] = robots[id].getCanvasPosition();
        addDirtyRegion(x, y, configOptions.visibility_radius);
      }
      requestAnimationFrame(redrawCanvas);
    }
  }

  return { gui, updatePauseText };
}

function startDrawingLoop() {
  stopAnimation = false;
  drawingSimulation = true;
  lastFrameTime = performance.now();
  requestAnimationFrame(drawLoop);
}

function stopDrawingLoop() {
  stopAnimation = true;
  drawingSimulation = false;
}

// Improve tracking of the TwoRobot algorithm state
// Add visual debugging for TwoRobot algorithm
function updateTwoRobotDebug() {
  // Only run for TwoRobot algorithm
  if (configOptions.algorithm !== "TwoRobot") return;
  
  // Create or update debug info
  let debugElem = document.getElementById('tworobot-debug');
  if (!debugElem) {
    debugElem = document.createElement('div');
    debugElem.id = 'tworobot-debug';
    debugElem.style.position = 'absolute';
    debugElem.style.top = '40px';
    debugElem.style.left = '10px';
    debugElem.style.backgroundColor = 'rgba(0,0,0,0.7)';
    debugElem.style.color = 'white';
    debugElem.style.padding = '5px';
    debugElem.style.borderRadius = '5px';
    debugElem.style.fontSize = '12px';
    document.body.appendChild(debugElem);
  }
  
  // Update debug info with robot states
  let robotInfo = '';
  for (const id in robots) {
    const robot = robots[id];
    robotInfo += `Robot ${id}: ${robot.getColor()} at (${robot.x.toFixed(2)}, ${robot.y.toFixed(2)})<br>`;
  }
  
  debugElem.innerHTML = robotInfo || 'Waiting for robots...';
}

// Add MutualVisibility debug info
function updateMutualVisibilityDebug() {
  // Only run for MutualVisibility algorithm
  if (configOptions.algorithm !== "MutualVisibility") return;
  
  // Create or update debug info
  let debugElem = document.getElementById('mutualvis-debug');
  if (!debugElem) {
    debugElem = document.createElement('div');
    debugElem.id = 'mutualvis-debug';
    debugElem.style.position = 'absolute';
    debugElem.style.top = '40px';
    debugElem.style.right = '10px';
    debugElem.style.backgroundColor = 'rgba(0,0,0,0.7)';
    debugElem.style.color = 'white';
    debugElem.style.padding = '10px';
    debugElem.style.borderRadius = '5px';
    debugElem.style.fontSize = '12px';
    debugElem.style.maxWidth = '300px';
    document.body.appendChild(debugElem);
  }
  
  // Update debug info with robot states
  let robotInfo = '<strong>MutualVisibility Debug:</strong><br>';
  let redCount = 0;
  let greenCount = 0;
  
  for (const id in robots) {
    const robot = robots[id];
    const color = robot.getColor().toUpperCase();
    if (color === "#FF0000") redCount++;
    if (color === "#00FF00") greenCount++;
    
    robotInfo += `Robot ${id}: ${color} at (${robot.x.toFixed(1)}, ${robot.y.toFixed(1)})<br>`;
  }
  
  robotInfo += '<br><strong>Summary:</strong><br>';
  robotInfo += `Red robots: ${redCount}<br>`;
  robotInfo += `Green robots: ${greenCount}<br>`;
  robotInfo += `Total robots: ${Object.keys(robots).length}`;
  
  debugElem.innerHTML = robotInfo;
}

function drawLoop(currentTime) {
  if (stopAnimation) return;
  if (simulationId === undefined) return;
  
  // FPS tracking
  frameCount++;
  if (currentTime - lastFpsTime >= 1000) {
    fps = Math.round((frameCount * 1000) / (currentTime - lastFpsTime));
    frameCount = 0;
    lastFpsTime = currentTime;
  }
  
  // Throttle rendering for performance
  const timeSinceLastRender = currentTime - lastRenderTime;
  if (timeSinceLastRender < RENDER_THROTTLE_MS && !needsViewportUpdate) {
    requestAnimationFrame(drawLoop);
    return;
  }
  
  const deltaTime = currentTime - lastFrameTime;
  if (deltaTime >= timePerFrameMs && !paused) {
    // Process multiple snapshots if we have a large queue, to catch up
    let snapshotsToProcess = 1;
    
    if (snapshotQueue.size > 5) {
      // If we're falling behind, process more snapshots at once
      snapshotsToProcess = Math.min(3, Math.floor(snapshotQueue.size / 2));
    }
    
    let processedAny = false;
    
    for (let i = 0; i < snapshotsToProcess; i++) {
      const snapshot = snapshotQueue.dequeue();
      if (snapshot) {
        drawSnapshot(snapshot);
        processedAny = true;
        
        // Only update timing on the last processed snapshot
        if (i === snapshotsToProcess - 1) {
          lastFrameTime = currentTime;
          lastRenderTime = currentTime;
        }
      } else {
        break; // No more snapshots
      }
    }
    
    if (!processedAny) {
      stopDrawingLoop();
      drawSEC(sec);
      return;
    }
  }
  
  // Update debug info
  if (drawingSimulation) {
    if (configOptions.algorithm === "MutualVisibility") {
      updateMutualVisibilityDebug();
    } else if (configOptions.algorithm === "TwoRobot") {
      updateTwoRobotDebug();
    }
  }
  
  // Draw FPS counter if enabled
  if (showFps) {
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.font = '12px Arial';
    ctx.fillStyle = 'black';
    ctx.fillText(`FPS: ${fps} | Queue: ${snapshotQueue.size}`, 10, 20);
    ctx.restore();
  }
  
  requestAnimationFrame(drawLoop);
}

function redrawCanvas() {
  if (needsViewportUpdate) {
    clearCanvas();
    
    // Redraw all robots
    for (const id in robots) {
      drawRobot(robots[id]);
    }
    
    needsViewportUpdate = false;
  } else {
    // Only clear and redraw dirty regions
    clearDirtyRegions();
    
    // Redraw robots in dirty regions
    for (const id in robots) {
      drawRobot(robots[id]);
    }
  }
}

function clearCanvas() {
  // Save current transform
  ctx.save();
  
  // Reset transform to identity
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  
  // Clear the entire canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  // Restore previous transform
  ctx.restore();
  
  // Clear dirty regions since we've cleared everything
  dirtyRegions.clear();
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
  
  // Reset transform
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  // Set default transform (centered coordinates)
  ctx.setTransform(1, 0, 0, 1, canvas.width / 2, canvas.height / 2);
  
  // Reset view parameters
  viewScale = 1.0;
  viewCenterX = 0;
  viewCenterY = 0;
  needsViewportUpdate = true;
  
  configOptions.width_bound = canvas.width / 2;
  configOptions.height_bound = canvas.height / 2;
  
  // Redraw if robots exist
  if (Object.keys(robots).length > 0) {
    redrawCanvas();
  }
}

function drawSnapshot(snapshot) {
  let timeVal = snapshot[0];
  updateTimeElement(timeVal);
  let robotsHistory = snapshot[1];
  
  // Find the min and max coordinates of all robots
  let minX = Infinity, minY = Infinity;
  let maxX = -Infinity, maxY = -Infinity;
  
  // First pass: find bounds of all robots
  for (let id in robotsHistory) {
    let [x, y] = robotsHistory[id][0];
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  
  // Add padding to ensure robots aren't at the edges
  const padding = 200;  // Increased padding for better visibility
  minX -= padding;
  minY -= padding;
  maxX += padding;
  maxY += padding;
  
  // Calculate the width and height of the bounding box
  const width = maxX - minX;
  const height = maxY - minY;
  
  // Calculate the center of the bounding box
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  
  // Set initial view only once at the beginning of simulation
  if (isFirstRender) {
    // Calculate the scale needed to fit all robots in view
    // Consider both canvas dimensions and give some margin
    const canvasWidth = canvas.width / Robot.ROBOT_X_POS_FACTOR;
    const canvasHeight = canvas.height / Robot.ROBOT_Y_POS_FACTOR;
    
    const scaleX = canvasWidth / width;
    const scaleY = canvasHeight / height;
    
    // Use the smaller scale to ensure everything fits
    viewScale = Math.max(Math.min(scaleX, scaleY, 1.0), 0.3); // Cap between 0.3 and 1.0
    
    // Set the view center to the center of all robots
    viewCenterX = centerX;
    viewCenterY = centerY;
    
    // Mark for viewport update
    needsViewportUpdate = true;
    isFirstRender = false;
    
    console.log(`Initial view set: center(${viewCenterX}, ${viewCenterY}), scale: ${viewScale}`);
  }
  
  // Apply the viewport adjustments
  if (needsViewportUpdate) {
    // Save current transform
    ctx.save();
    
    // Reset transform to identity
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    
    // Clear the canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Set new transform that centers and scales the view
    ctx.setTransform(
      viewScale, 0, 
      0, viewScale, 
      canvas.width/2 - viewCenterX * viewScale * Robot.ROBOT_X_POS_FACTOR, 
      canvas.height/2 - viewCenterY * viewScale * Robot.ROBOT_Y_POS_FACTOR
    );
    
    dirtyRegions.clear(); // Clear dirty regions since we're redrawing everything
  }
  
  // Clear dirty regions or entire canvas based on need
  if (!needsViewportUpdate) {
    clearDirtyRegions();
  }
  
  // Now draw all robots with the adjusted viewport
  for (let id in robotsHistory) {
    let [x, y] = robotsHistory[id][0];
    const state = robotsHistory[id][1];
    const multiplicity = robotsHistory[id][4];
    const light = robotsHistory[id][5];
    
    // Track robot movement to create dirty regions
    if (robotPrevPositions[id]) {
      const [prevX, prevY] = robotPrevPositions[id];
      const moveDistance = Math.hypot(x - prevX, y - prevY);
      
              // If the robot moved, add dirty regions for both old and new positions
      if (moveDistance > 0) {
        // Add previous position as a dirty region (scaled to canvas)
        addDirtyRegion(
          prevX * Robot.ROBOT_X_POS_FACTOR, 
          prevY * Robot.ROBOT_Y_POS_FACTOR,
          Math.max(Robot.ROBOT_SIZE * 2, configOptions.visibility_radius || 20)
        );
      }
    }
    
    // Update robot in our tracking
    if (robots[id] === undefined) {
      robots[id] = new Robot(x, y, id, light, 1, multiplicity);
    } else {
      robots[id].setColor(light);
    }
    
    robots[id].setPosition(x, y);
    robots[id].setState(state);
    robots[id].multiplicity = multiplicity;
    
    // Store current position for next frame
    robotPrevPositions[id] = [x, y];
    
    // Draw the robot
    drawRobot(robots[id]);
  }
  
  // Reset viewport update flag
  if (needsViewportUpdate) {
    needsViewportUpdate = false;
  }
}

function updateTimeElement(t) {
  time.innerText = t;
}

function handleCanvasClick(e) {
  if (time.innerText !== "") {
    clearSimulation();
  }
  
  // Get click coordinates in canvas space
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  
  // Convert to canvas pixel coordinates
  const canvasX = (e.clientX - rect.left) * scaleX;
  const canvasY = (e.clientY - rect.top) * scaleY;
  
  // Get current transform and invert it to get world coordinates
  const transform = ctx.getTransform();
  const inverseTransform = transform.invertSelf();
  
  // Apply inverse transform to get world coordinates
  const worldX = canvasX * inverseTransform.a + canvasY * inverseTransform.c + inverseTransform.e;
  const worldY = canvasX * inverseTransform.b + canvasY * inverseTransform.d + inverseTransform.f;
  
  // Create robot at clicked world position
  const robot = new Robot(worldX, worldY, `${currRobotId++}`, configOptions.robot_colors, configOptions.robot_speeds, 1, true);
  robots[currRobotId - 1] = robot;
  
  // Mark this area as dirty for redrawing
  const [x, y] = robot.getCanvasPosition();
  addDirtyRegion(x, y, Robot.ROBOT_SIZE * 2);
  
  // Draw the robot
  drawRobot(robot);
  
  // Store the position
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
  
  // Reset viewport parameters
  viewScale = 1.0;
  viewCenterX = 0;
  viewCenterY = 0;
  needsViewportUpdate = true;
  
  // Reset optimization tracking
  dirtyRegions.clear();
  robotPrevPositions = {};
  isFirstRender = true;
  
  // Reset transform to default
  ctx.setTransform(1, 0, 0, 1, canvas.width / 2, canvas.height / 2);
  
  // Clear debug elements
  const twoRobotDebug = document.getElementById('tworobot-debug');
  if (twoRobotDebug) {
    twoRobotDebug.remove();
  }
  
  const mutualVisDebug = document.getElementById('mutualvis-debug');
  if (mutualVisDebug) {
    mutualVisDebug.remove();
  }
}

function translateToCanvas(canvas, x, y) {
  return [x - canvas.width / 2, y - canvas.height / 2];
}

// Initialize Web Worker for data processing
function initializeDataWorker() {
  try {
    // Create a Blob URL to load the worker script
    if (document.querySelector('#worker-script')) {
      const workerBlob = new Blob([
        document.querySelector('#worker-script').textContent
      ], { type: 'application/javascript' });
      
      dataWorker = new Worker(URL.createObjectURL(workerBlob));
      
      // Set up message event handlers
      dataWorker.onmessage = function(e) {
        const { type, data } = e.data;
        
        switch(type) {
          case 'batch_processed':
            // Handle processed batch data
            processBatchResults(data);
            break;
            
          case 'stats_calculated':
            // Handle calculated stats
            updateExtendedStats(data);
            break;
            
          case 'cache_cleared':
            console.log('Worker cache cleared');
            break;
        }
      };
      
      console.log('Data processing worker initialized');
    } else {
      console.warn('Worker script element not found');
      dataWorker = null;
    }
  } catch (err) {
    console.error('Failed to initialize data worker:', err);
    // Fallback to synchronous processing
    dataWorker = null;
  }
}

// Process batch results from worker
function processBatchResults(processedData) {
  if (!processedData || processedData.length === 0) return;
  
  // Apply optimizations based on worker calculations
  processedData.forEach(item => {
    if (item.needsFullRedraw) {
      needsViewportUpdate = true;
    }
    
    // Process the snapshot
    if (simulationId === item.simulation_id) {
      startDrawingLoop();
      snapshotQueue.enqueue(item.snapshot);
    }
  });
}

// Update extended stats from worker
function updateExtendedStats(extendedStats) {
  if (!extendedStats) return;
  
  // If stats visualization is integrated, update it with extended stats
  if (window.updateSimulationStats) {
    window.updateSimulationStats(extendedStats);
  }
  
  // Log extended stats
  console.log('Extended simulation statistics:', extendedStats);
}

// Initialize worker when the page loads
window.addEventListener('load', function() {
  // Wait a moment for other scripts to load
  setTimeout(initializeDataWorker, 500);
});

export {configOptions, resizeCanvas};