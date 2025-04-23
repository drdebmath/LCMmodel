/* main.js – updated so Pyodide can import robot/scheduler/run correctly */

console.log("main.js started");

/* ------------------------------------------------------------------ *
 *  DOM ELEMENT REFERENCES
 * ------------------------------------------------------------------ */
const statusDiv            = document.getElementById("status");
const startButton          = document.getElementById("start_button");
const stopButton           = document.getElementById("stop_button");
const canvas               = document.getElementById("canvas");
const ctx                  = canvas.getContext("2d");
const simTimeDiv           = document.getElementById("sim_time");
const simMessageDiv        = document.getElementById("sim_message");

const algorithmSelect      = document.getElementById("algorithm");
const numRobotsSlider      = document.getElementById("num_robots");
const numRobotsVal         = document.getElementById("num_robots_val");
const robotSpeedSlider     = document.getElementById("robot_speed");
const robotSpeedVal        = document.getElementById("robot_speed_val");
const visibilityRadiusSlider   = document.getElementById("visibility_radius");
const visibilityRadiusVal      = document.getElementById("visibility_radius_val");
const infiniteVisibilityCheckbox = document.getElementById("infinite_visibility");
const numFaultsSlider      = document.getElementById("num_faults");
const numFaultsVal         = document.getElementById("num_faults_val");
const rigidMovementCheckbox = document.getElementById("rigid_movement");
const widthBoundSlider     = document.getElementById("width_bound");
const widthBoundVal        = document.getElementById("width_bound_val");
const heightBoundSlider    = document.getElementById("height_bound");
const heightBoundVal       = document.getElementById("height_bound_val");
const lambdaRateSlider     = document.getElementById("lambda_rate");
const lambdaRateVal        = document.getElementById("lambda_rate_val");
const samplingRateSlider   = document.getElementById("sampling_rate");
const samplingRateVal      = document.getElementById("sampling_rate_val");
const thresholdPrecisionSlider  = document.getElementById("threshold_precision");
const thresholdPrecisionVal     = document.getElementById("threshold_precision_val");
const randomSeedInput      = document.getElementById("random_seed");

/* ------------------------------------------------------------------ *
 *  PYODIDE / SIMULATION STATE
 * ------------------------------------------------------------------ */
let pyodide          = null;
let pythonRunner     = null;     // proxy to Python module “simulation_runner”
let simulationRunning = false;
let animationFrameId  = null;

/* ------------------------------------------------------------------ *
 *  CANVAS CONFIG
 * ------------------------------------------------------------------ */
const robotRadius = 5;
let scale   = 2;
let offsetX = 0;
let offsetY = 0;
const colors = [
  "#FF0000", "#0000FF", "#008000", "#FFA500", "#800080",
  "#00FFFF", "#FF00FF", "#4682B4", "#FFD700", "#32CD32",
];

/* ------------------------------------------------------------------ *
 *  SMALL HELPERS
 * ------------------------------------------------------------------ */
function updateStatus(msg) {
  console.log("Status:", msg);
  if (statusDiv) statusDiv.textContent = msg;
}

function updateSimMessage(msg) {
  console.log("SimMessage:", msg);
  if (!simMessageDiv) return;
  simMessageDiv.textContent = msg;
  if (msg && msg.toLowerCase().includes("error")) {
    simMessageDiv.style.color      = "red";
    simMessageDiv.style.fontWeight = "bold";
  } else {
    simMessageDiv.style.color      = "black";
    simMessageDiv.style.fontWeight = "normal";
  }
}

function updateSliderValue(slider, span) {
  if (slider && span) span.textContent = slider.value;
}

// --- Canvas resize (replace the previous version) ------------------
function resizeCanvas() {
    const viz = document.getElementById("visualization");
    if (!viz) return;
  
    /* use exact CSS pixel size, no magic “-20” paddings */
    const { width, height } = viz.getBoundingClientRect();
  
    /* physical bitmap size for crispness on Hi-DPI screens         */
    const dpr = window.devicePixelRatio || 1;
    canvas.width  = Math.floor(width  * dpr);
    canvas.height = Math.floor(height * dpr);
  
    /* show it at 100% logical size                                  */
    canvas.style.width  = `${width}px`;
    canvas.style.height = `${height}px`;
  
    offsetX = width  / 2;
    offsetY = height / 2;
  
    /* world-bound scaling (unchanged) */
    const worldW = +widthBoundSlider.value;
    const worldH = +heightBoundSlider.value;
    scale = Math.min(width / worldW, height / worldH) * 0.9;
  
    console.log(`Canvas resized: ${width}×${height}  (dpr ${dpr})`);
  }
  

/* ------------------------------------------------------------------ *
 *  PYODIDE BOOTSTRAP
 * ------------------------------------------------------------------ */
async function loadPyodideAndPackages() {
  updateStatus("Loading Pyodide runtime…");
  try {
    pyodide = await loadPyodide();
    await pyodide.loadPackage(["numpy", "micropip"]);
    updateStatus("Runtime ready. Loading Python files…");

    /* 1️⃣  Fetch and write files into the in-browser FS            */
    const pyFiles = ["robot.py", "scheduler.py", "run.py"];
    for (const fname of pyFiles) {
      updateStatus(`Fetching ${fname}…`);
      const res = await fetch(`./${fname}`);
      if (!res.ok) throw new Error(`Fetch failed for ${fname}: ${res.statusText}`);
      const code = await res.text();
      pyodide.FS.writeFile(fname, code);
    }

    /* 2️⃣  Import “run” → register as simulation_runner           */
    updateStatus("Importing Python modules…");
    await pyodide.runPythonAsync("import run as simulation_runner");

    /* 3️⃣  Grab proxy to the module                               */
    pythonRunner = pyodide.globals.get("simulation_runner");

    updateStatus("Python ready. You can start the simulation.");
    startButton.disabled = false;
  } catch (err) {
    console.error(err);
    updateStatus(`Pyodide init error: ${err.message}`);
  }
}

/* ------------------------------------------------------------------ *
 *  SIMULATION CONTROL
 * ------------------------------------------------------------------ */
async function startSimulation() {
  if (simulationRunning || !pythonRunner) return;

  startButton.disabled = true;
  stopButton.disabled  = false;
  simulationRunning    = true;
  updateSimMessage("");

  const params = {
    algorithm          : algorithmSelect.value,
    num_of_robots      : +numRobotsSlider.value,
    robot_speeds       : +robotSpeedSlider.value,
    visibility_radius  : infiniteVisibilityCheckbox.checked ? null : +visibilityRadiusSlider.value,
    num_of_faults      : +numFaultsSlider.value,
    rigid_movement     : rigidMovementCheckbox.checked,
    width_bound        : +widthBoundSlider.value,
    height_bound       : +heightBoundSlider.value,
    lambda_rate        : +lambdaRateSlider.value,
    sampling_rate      : +samplingRateSlider.value,
    threshold_precision: +thresholdPrecisionSlider.value,
    random_seed        : +randomSeedInput.value,
    initial_positions  : [],
  };

  resizeCanvas();

  try {
    const setupJson   = await pythonRunner.setup_simulation(JSON.stringify(params));
    const setupResult = JSON.parse(setupJson);

    if (setupResult.status === "error") throw new Error(setupResult.message);

    updateStatus("Simulation running…");
    updateSimMessage(setupResult.message || "Running…");
    simTimeDiv.textContent = `Time: ${setupResult.time.toFixed(2)}`;
    drawSimulation(setupResult.robots);
    animationFrameId = requestAnimationFrame(simulationStep);
  } catch (err) {
    console.error(err);
    updateStatus("Failed to start simulation.");
    updateSimMessage(`Error: ${err.message}`);
    startButton.disabled = false;
    stopButton.disabled  = true;
    simulationRunning    = false;
  }
}

function stopSimulation() {
  if (!simulationRunning) return;
  simulationRunning = false;
  cancelAnimationFrame(animationFrameId);
  animationFrameId = null;

  try {
    const stopJson   = pythonRunner.stop_simulation();
    const stopResult = JSON.parse(stopJson);
    updateSimMessage(stopResult.message);
  } catch (e) {
    console.error("stop_simulation error:", e);
  }

  updateStatus("Simulation stopped.");
  startButton.disabled = false;
  stopButton.disabled  = true;
}

async function simulationStep() {
  if (!simulationRunning) return;

  try {
    const stepJson = await pythonRunner.run_simulation_step();
    const step     = JSON.parse(stepJson);

    simTimeDiv.textContent = `Time: ${step.time.toFixed(2)}`;
    if (step.message) updateSimMessage(step.message);
    if (step.robots)  drawSimulation(step.robots);

    if (step.status !== "running") {
      simulationRunning   = false;
      startButton.disabled = false;
      stopButton.disabled  = true;
      updateStatus(`Simulation ${step.status}.`);
    }
  } catch (err) {
    console.error("Step error:", err);
    updateStatus("Simulation error!");
    updateSimMessage(`Runtime error: ${err.message}`);
    simulationRunning    = false;
    startButton.disabled = false;
    stopButton.disabled  = true;
  }

  if (simulationRunning) animationFrameId = requestAnimationFrame(simulationStep);
}

/* ------------------------------------------------------------------ *
 *  DRAWING UTILS
 * ------------------------------------------------------------------ */
function transformCoords(x, y) {
  return { x: offsetX + x * scale, y: offsetY - y * scale };
}

function drawSimulation(robots) {
  if (!ctx) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  /* world rectangle */
  const w = +widthBoundSlider.value;
  const h = +heightBoundSlider.value;
  const tl = transformCoords(-w / 2,  h / 2);
  const br = transformCoords( w / 2, -h / 2);
  ctx.strokeStyle = "#ccc";
  ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

  /* robots */
  robots.forEach((r) => {
    const p = transformCoords(r.x, r.y);
    ctx.fillStyle   = r.crashed ? "#404040"
                     : r.terminated ? "#a0a0a0"
                     : colors[r.id % colors.length];
    ctx.strokeStyle = "#000";
    ctx.beginPath();
    ctx.arc(p.x, p.y, robotRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    /* id / state label */
    let label = `${r.id}`;
    if (r.state === "MOVE") label += " M";
    if (r.frozen)           label += "*";
    if (r.terminated)       label += "#";
    if (r.crashed)          label += "!";
    if (r.multiplicity > 1) label += `(${r.multiplicity})`;
    ctx.fillStyle = "#000";
    ctx.font      = "10px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(label, p.x, p.y + robotRadius + 10);

    /* target line */
    if (r.state === "MOVE" && r.target_x != null) {
      const t = transformCoords(r.target_x, r.target_y);
      ctx.setLineDash([2, 3]);
      ctx.strokeStyle = ctx.fillStyle;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(t.x, t.y);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    /* SEC (SEC algorithm only) */
    if (algorithmSelect.value === "SEC" && r.sec) {
      const center = transformCoords(r.sec.center_x, r.sec.center_y);
      ctx.strokeStyle = "rgba(255,165,0,0.6)";
      ctx.beginPath();
      ctx.arc(center.x, center.y, r.sec.radius * scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    /* visibility circle */
    if (r.visibility_radius && !r.crashed && !r.terminated) {
      ctx.strokeStyle = "rgba(100,100,200,0.3)";
      ctx.beginPath();
      ctx.arc(p.x, p.y, r.visibility_radius * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
  });
}

/* ------------------------------------------------------------------ *
 *  EVENT LISTENERS
 * ------------------------------------------------------------------ */
startButton.addEventListener("click", startSimulation);
stopButton .addEventListener("click", stopSimulation);
window.addEventListener("resize", resizeCanvas);

numRobotsSlider     .addEventListener("input", () => updateSliderValue(numRobotsSlider, numRobotsVal));
robotSpeedSlider    .addEventListener("input", () => updateSliderValue(robotSpeedSlider, robotSpeedVal));
visibilityRadiusSlider.addEventListener("input", () => updateSliderValue(visibilityRadiusSlider, visibilityRadiusVal));
numFaultsSlider     .addEventListener("input", () => updateSliderValue(numFaultsSlider, numFaultsVal));
widthBoundSlider    .addEventListener("input", () => { updateSliderValue(widthBoundSlider,  widthBoundVal);  resizeCanvas(); });
heightBoundSlider   .addEventListener("input", () => { updateSliderValue(heightBoundSlider, heightBoundVal); resizeCanvas(); });
lambdaRateSlider    .addEventListener("input", () => updateSliderValue(lambdaRateSlider,  lambdaRateVal));
samplingRateSlider  .addEventListener("input", () => updateSliderValue(samplingRateSlider, samplingRateVal));
thresholdPrecisionSlider.addEventListener("input", () => updateSliderValue(thresholdPrecisionSlider, thresholdPrecisionVal));

infiniteVisibilityCheckbox.addEventListener("change", () => {
  visibilityRadiusSlider.disabled = infiniteVisibilityCheckbox.checked;
  visibilityRadiusVal.textContent =
    infiniteVisibilityCheckbox.checked ? "Inf" : visibilityRadiusSlider.value;
});

/* ------------------------------------------------------------------ *
 *  INITIAL SETUP
 * ------------------------------------------------------------------ */
[
  [numRobotsSlider,          numRobotsVal],
  [robotSpeedSlider,         robotSpeedVal],
  [visibilityRadiusSlider,   visibilityRadiusVal],
  [numFaultsSlider,          numFaultsVal],
  [widthBoundSlider,         widthBoundVal],
  [heightBoundSlider,        heightBoundVal],
  [lambdaRateSlider,         lambdaRateVal],
  [samplingRateSlider,       samplingRateVal],
  [thresholdPrecisionSlider, thresholdPrecisionVal],
].forEach(([slider, span]) => updateSliderValue(slider, span));

if (infiniteVisibilityCheckbox.checked) {
  visibilityRadiusSlider.disabled = true;
  visibilityRadiusVal.textContent = "Inf";
}

resizeCanvas();
drawSimulation([]);

/* boot the runtime */
loadPyodideAndPackages();

console.log("main.js loaded and setup complete.");
