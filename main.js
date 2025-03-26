// main.js - Guaranteed to show visual feedback
console.log("main.js loaded successfully");
window.mainJsLoaded = true; // For the fallback detection

// Initialize
const statusElement = document.getElementById('status');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// Set canvas to full window size
function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  drawInitialState();
}

// Initial drawing
function drawInitialState() {
  ctx.fillStyle = '#333';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  ctx.fillStyle = '#0f0';
  ctx.font = '24px Arial';
  ctx.fillText('LCM Visualization Ready', 50, 50);
  
  ctx.fillStyle = '#fff';
  ctx.font = '16px Arial';
  ctx.fillText(`Canvas: ${canvas.width}x${canvas.height}`, 50, 80);
  
  statusElement.textContent = "Waiting for robot data...";
}

// WebSocket connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = window.location.host;
  const wsUrl = `${protocol}//${host}/LCMmodel/ws`;
  
  const socket = new WebSocket(wsUrl);
  
  socket.onopen = () => {
    statusElement.textContent = "Connected to simulator";
    console.log("WebSocket connected");
  };
  
  socket.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateVisualization(data);
    } catch (e) {
      console.error("Error parsing data:", e);
    }
  };
  
  socket.onerror = (error) => {
    statusElement.textContent = "Connection error";
    console.error("WebSocket error:", error);
  };
  
  socket.onclose = () => {
    statusElement.textContent = "Disconnected";
    console.log("WebSocket closed");
  };
}

// Update visualization with robot data
function updateVisualization(data) {
  // Clear canvas
  ctx.fillStyle = '#222';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  // Draw each robot
  data.robots.forEach(robot => {
    const x = robot.x * 10 + canvas.width/2; // Scale and center
    const y = robot.y * 10 + canvas.height/2;
    
    // Set color based on fault state
    ctx.fillStyle = getRobotColor(robot.fault);
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, Math.PI * 2);
    ctx.fill();
    
    // Draw direction indicator if available
    if (robot.direction) {
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(
        x + Math.cos(robot.direction) * 15,
        y + Math.sin(robot.direction) * 15
      );
      ctx.stroke();
    }
  });
  
  statusElement.textContent = `Showing ${data.robots.length} robots`;
}

// Color mapping
function getRobotColor(faultType) {
  const colors = {
    'CRASH': '#ff0000',
    'BYZANTINE': '#ff9900',
    'DELAY': '#ffff00',
    'NORMAL': '#00aaff',
    'OMISSION': '#9900ff'
  };
  return colors[faultType] || '#ffffff';
}

// Start everything
resizeCanvas();
window.addEventListener('resize', resizeCanvas);
connectWebSocket();

// Initial test drawing
setTimeout(() => {
  if (document.getElementById('status').textContent.includes("Waiting")) {
    // Simulate test data if no connection
    updateVisualization({
      robots: [
        {x: -2, y: -1, fault: 'NORMAL'},
        {x: 1, y: 1, fault: 'CRASH'},
        {x: 0, y: 0, fault: 'BYZANTINE'}
      ]
    });
    statusElement.textContent = "Using test data (no connection)";
  }
}, 3000);
