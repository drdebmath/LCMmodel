// main.js - LCM Robot Visualization
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusDiv = document.getElementById('status');

// Configuration
const ROBOT_RADIUS = 8;
const SCALE_FACTOR = 15; // Adjust based on your coordinate system
const FAULT_COLORS = {
    'CRASH': '#ff0000',
    'BYZANTINE': '#ff9900',
    'DELAY': '#ffff00',
    'NORMAL': '#00aaff',
    'OMISSION': '#9900ff'
};

// Initialize
function init() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    statusDiv.textContent = "Connecting to simulator...";
    connectWebSocket();
}

// WebSocket Connection
function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.hostname}:8001`;
    
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        statusDiv.textContent = "Connected to simulator";
    };

    socket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            drawRobots(data.robots);
            statusDiv.textContent = `Robots: ${data.robots.length} | FPS: ${calculateFPS()}`;
        } catch (e) {
            console.error("Error parsing data:", e);
        }
    };

    socket.onerror = (error) => {
        statusDiv.textContent = "Connection error";
        console.error("WebSocket error:", error);
    };
}

// Drawing Functions
function drawRobots(robots) {
    // Clear canvas
    ctx.fillStyle = '#222222';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Calculate center offset
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    
    // Draw each robot
    robots.forEach(robot => {
        const x = robot.x * SCALE_FACTOR + centerX;
        const y = robot.y * SCALE_FACTOR + centerY;
        
        // Draw robot body
        ctx.fillStyle = FAULT_COLORS[robot.fault] || '#ffffff';
        ctx.beginPath();
        ctx.arc(x, y, ROBOT_RADIUS, 0, Math.PI * 2);
        ctx.fill();
        
        // Draw direction indicator
        if (robot.direction) {
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(
                x + Math.cos(robot.direction) * (ROBOT_RADIUS + 5),
                y + Math.sin(robot.direction) * (ROBOT_RADIUS + 5)
            );
            ctx.stroke();
        }
    });
}

// FPS Counter
let lastTime = 0;
let frameCount = 0;
function calculateFPS() {
    frameCount++;
    const now = performance.now();
    if (now >= lastTime + 1000) {
        const fps = Math.round((frameCount * 1000) / (now - lastTime));
        lastTime = now;
        frameCount = 0;
        return fps;
    }
}

// Start visualization
window.addEventListener('load', init);
window.addEventListener('resize', init);
