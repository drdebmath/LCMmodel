// main.js - Basic setup
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const loading = document.getElementById('loading');

function init() {
  // Remove loading message
  loading.style.display = 'none';
  
  // Set canvas to full window size
  resizeCanvas();
  
  // Draw test content
  ctx.fillStyle = '#4285F4';
  ctx.fillRect(100, 100, 200, 100);
  
  ctx.fillStyle = '#EA4335';
  ctx.beginPath();
  ctx.arc(300, 300, 50, 0, Math.PI * 2);
  ctx.fill();
  
  console.log('Visualization ready!');
}

function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
}

// Event listeners
window.addEventListener('load', init);
window.addEventListener('resize', resizeCanvas);
