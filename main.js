/* ================================================================
 *  main.js  —  robots coloured by state, no number labels
 * ================================================================ */

/* -------- DOM refs ------------------------------------------------ */
const statusDiv     = document.getElementById("status");
const startBtn      = document.getElementById("start_button");
const stopBtn       = document.getElementById("stop_button");
const zoomInBtn     = document.getElementById("zoom_in_btn");
const zoomOutBtn    = document.getElementById("zoom_out_btn");

const canvas        = document.getElementById("canvas");
const ctx           = canvas.getContext("2d");

const simTimeDiv    = document.getElementById("sim_time");
const simMsgDiv     = document.getElementById("sim_message");

/* sliders / fields (unchanged from previous version) -------------- */
const algorithmSelect = document.getElementById("algorithm");
const numRobotsSlider = document.getElementById("num_robots");
const numRobotsVal    = document.getElementById("num_robots_val");
const robotSpeedSlider = document.getElementById("robot_speed");
const robotSpeedVal    = document.getElementById("robot_speed_val");

const visibilitySlider  = document.getElementById("visibility_radius");
const visibilityVal     = document.getElementById("visibility_radius_val");
const infiniteVisChk    = document.getElementById("infinite_visibility");

const numFaultsSlider   = document.getElementById("num_faults");
const numFaultsVal      = document.getElementById("num_faults_val");
const rigidChk          = document.getElementById("rigid_movement");

const widthSlider       = document.getElementById("width_bound");
const widthVal          = document.getElementById("width_bound_val");
const heightSlider      = document.getElementById("height_bound");
const heightVal         = document.getElementById("height_bound_val");

const lambdaSlider      = document.getElementById("lambda_rate");
const lambdaVal         = document.getElementById("lambda_rate_val");
const sampleSlider      = document.getElementById("sampling_rate");
const sampleVal         = document.getElementById("sampling_rate_val");

const precisionSlider   = document.getElementById("threshold_precision");
const precisionVal      = document.getElementById("threshold_precision_val");

const seedInput         = document.getElementById("random_seed");

/* -------- Pyodide / sim state ------------------------------------ */
let pyodide      = null;
let pythonRunner = null;
let simRunning   = false;
let animFrameId  = null;

/* -------- canvas / world config ---------------------------------- */
const ROBOT_R = 5;
let scale   = 1;
let offsetX = 0, offsetY = 0;

const ZOOM_STEP = 1.15;
const MIN_SCALE = 0.05;
const MAX_SCALE = 50;

let lastRobotsFrame = [];

/* -------- helpers ------------------------------------------------ */
const updStatus = (msg) => (statusDiv.textContent = msg);
function updMsg(msg) {
  simMsgDiv.textContent = msg;
  simMsgDiv.style.color       = msg.toLowerCase().includes("error") ? "red" : "";
  simMsgDiv.style.fontWeight  = msg.toLowerCase().includes("error") ? "600" : "";
}
const updSlider = (sl, out) => (out.textContent = sl.value);

/* -------- canvas auto-size --------------------------------------- */
function resizeCanvas() {
  const vis = document.getElementById("visualization");
  if (!vis) return;

  const { width, height } = vis.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width  = Math.floor(width  * dpr);
  canvas.height = Math.floor(height * dpr);
  canvas.style.width  = `${width}px`;
  canvas.style.height = `${height}px`;

  offsetX = width / 2;
  offsetY = height / 2;

  const w = +widthSlider.value, h = +heightSlider.value;
  scale = Math.min(width / w, height / h) * 0.9;

  if (lastRobotsFrame.length) drawSimulation(lastRobotsFrame);
}

/* -------- zoom buttons ------------------------------------------- */
function applyZoom(factor) {
  scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, scale * factor));
  if (lastRobotsFrame.length) drawSimulation(lastRobotsFrame);
}
zoomInBtn .addEventListener("click", () => applyZoom(ZOOM_STEP));
zoomOutBtn.addEventListener("click", () => applyZoom(1 / ZOOM_STEP));

/* -------- Pyodide bootstrap -------------------------------------- */
async function loadPyodideAndPackages() {
  updStatus("Loading Pyodide…");
  try {
    pyodide = await loadPyodide();
    await pyodide.loadPackage(["numpy", "micropip"]);
    const files = ["robot.py", "scheduler.py", "run.py"];
    for (const f of files) {
      const code = await (await fetch(`./${f}`)).text();
      pyodide.FS.writeFile(f, code);
    }
    await pyodide.runPythonAsync("import run as simulation_runner");
    pythonRunner = pyodide.globals.get("simulation_runner");

    updStatus("Ready. Click Start.");
    startBtn.disabled = false;
  } catch (err) {
    updStatus(`Pyodide init error: ${err.message}`);
  }
}

/* -------- simulation control ------------------------------------- */
function params() {
  return {
    algorithm          : algorithmSelect.value,
    num_of_robots      : +numRobotsSlider.value,
    robot_speeds       : +robotSpeedSlider.value,
    visibility_radius  : infiniteVisChk.checked ? null : +visibilitySlider.value,
    num_of_faults      : +numFaultsSlider.value,
    rigid_movement     : rigidChk.checked,
    width_bound        : +widthSlider.value,
    height_bound       : +heightSlider.value,
    lambda_rate        : +lambdaSlider.value,
    sampling_rate      : +sampleSlider.value,
    threshold_precision: +precisionSlider.value,
    random_seed        : +seedInput.value,
    initial_positions  : []
  };
}

async function startSimulation() {
  if (simRunning || !pythonRunner) return;
  startBtn.disabled = true; stopBtn.disabled = false; simRunning = true; updMsg("");
  try {
    const setup = JSON.parse(await pythonRunner.setup_simulation(JSON.stringify(params())));
    if (setup.status === "error") throw new Error(setup.message);
    simTimeDiv.textContent = `Time: ${setup.time.toFixed(2)}`;
    drawSimulation(setup.robots); lastRobotsFrame = setup.robots;
    updStatus("Simulation running…");
    animFrameId = requestAnimationFrame(simStep);
  } catch (err) {
    updStatus("Start error."); updMsg(`Error: ${err.message}`);
    startBtn.disabled = false; stopBtn.disabled = true; simRunning = false;
  }
}

function stopSimulation() {
  if (!simRunning) return;
  simRunning = false; cancelAnimationFrame(animFrameId);
  try { updMsg(JSON.parse(pythonRunner.stop_simulation()).message); } catch {}
  updStatus("Simulation stopped."); startBtn.disabled = false; stopBtn.disabled = true;
}

/* ============================ main.js ============================ */
/* --- replace the whole simStep() with this version -------------- */
async function simStep() {
    if (!simRunning) return;
    try {
      const step = JSON.parse(await pythonRunner.run_simulation_step());
  
      /* update wall-clock time + UI msg */
      simTimeDiv.textContent = `Time: ${step.time.toFixed(2)}`;
      if (step.message) updMsg(step.message);
  
      /* redraw **only** on VISUALIZE (exit_code 99) or when simulation stops */
      const shouldRedraw = step.exit_code === 99 || step.status !== "running";
      if (shouldRedraw && step.robots) {
        drawSimulation(step.robots);
        lastRobotsFrame = step.robots;
      }
  
      /* stop loop if sim finished */
      if (step.status !== "running") {
        simRunning = false;
        updStatus(`Simulation ${step.status}.`);
        startBtn.disabled = false;
        stopBtn.disabled  = true;
        return;
      }
    } catch (err) {
      updStatus("Runtime error!");
      updMsg(`Error: ${err.message}`);
      simRunning = false;
      startBtn.disabled = false;
      stopBtn.disabled  = true;
    }
    if (simRunning) animFrameId = requestAnimationFrame(simStep);
  }
  

/* -------- drawing ------------------------------------------------ */
const STATE_COLORS = {
  CRASH      : "#555555",
  TERMINATED : "#a0a0a0",
  FROZEN     : "#ff9500",
  MOVE       : "#34c759",
  DEFAULT    : "#007aff"    // LOOK / WAIT / others
};
/* ---------- build legend (once) --------------------------------- */
function buildLegend() {
    const legendDiv = document.getElementById("legend");
    if (!legendDiv) return;
  
    const LABELS = {
      DEFAULT   : "Active (LOOK / WAIT)",
      MOVE      : "Moving",
      FROZEN    : "Frozen",
      TERMINATED: "Terminated",
      CRASH     : "Crashed"
    };
  
    Object.entries(STATE_COLORS).forEach(([key, hex]) => {
      if (!LABELS[key]) return;                // skip unknown keys
      const item   = document.createElement("div");
      item.className = "legend-item";
      item.innerHTML =
        `<span class="inline-block mr-2 rounded-sm"
          style="background:${hex};width:1rem;height:1rem;"></span>${LABELS[key]}`;
      legendDiv.appendChild(item);
    });
  }
  
function colorForRobot(r) {
  if (r.crashed)       return STATE_COLORS.CRASH;
  if (r.terminated)    return STATE_COLORS.TERMINATED;
  if (r.frozen)        return STATE_COLORS.FROZEN;
  if (r.state === "MOVE") return STATE_COLORS.MOVE;
  return STATE_COLORS.DEFAULT;
}

function transform(x, y) { return { x: offsetX + x * scale, y: offsetY - y * scale }; }

function drawSimulation(robots) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  robots.forEach((r) => {
    const p = transform(r.x, r.y);
    const c = colorForRobot(r);

    /* body */
    ctx.fillStyle = c; ctx.strokeStyle = "#000"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(p.x, p.y, ROBOT_R, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

    /* target line */
    if (r.state === "MOVE" && r.target_x != null) {
      const t = transform(r.target_x, r.target_y);
      ctx.setLineDash([2, 3]); ctx.strokeStyle = c; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(t.x, t.y); ctx.stroke(); ctx.setLineDash([]);
    }

    /* SEC (SEC algorithm only) */
    if (algorithmSelect.value === "SEC" && r.sec) {
      const sc = transform(r.sec.center_x, r.sec.center_y);
      ctx.strokeStyle = "rgba(255,165,0,0.6)"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(sc.x, sc.y, r.sec.radius * scale, 0, Math.PI * 2); ctx.stroke();
    }

    /* visibility */
    if (r.visibility_radius && !r.crashed && !r.terminated) {
      ctx.strokeStyle = "rgba(100,100,200,.25)"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(p.x, p.y, r.visibility_radius * scale, 0, Math.PI * 2); ctx.stroke();
    }
  });
}

/* -------- listeners & init -------------------------------------- */
startBtn.addEventListener("click", startSimulation);
stopBtn .addEventListener("click", stopSimulation);
window.addEventListener("resize", resizeCanvas);

[
  [numRobotsSlider, numRobotsVal], [robotSpeedSlider, robotSpeedVal],
  [visibilitySlider, visibilityVal], [numFaultsSlider, numFaultsVal],
  [widthSlider, widthVal], [heightSlider, heightVal],
  [lambdaSlider, lambdaVal], [sampleSlider, sampleVal],
  [precisionSlider, precisionVal]
].forEach(([sl, out]) => {
  updSlider(sl, out); sl.addEventListener("input", () => updSlider(sl, out));
});

infiniteVisChk.addEventListener("change", () => {
  visibilitySlider.disabled = infiniteVisChk.checked;
  visibilityVal.textContent = infiniteVisChk.checked ? "Inf" : visibilitySlider.value;
});

/* First-time layout */
buildLegend(); 
resizeCanvas(); 
drawSimulation([]);
loadPyodideAndPackages();
console.log("main.js loaded.");
