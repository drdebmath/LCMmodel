// FORCE-VISIBLE TEST (should work immediately)
document.addEventListener('DOMContentLoaded', () => {
  const canvas = document.getElementById('canvas');
  const ctx = canvas.getContext('2d');
  
  // Make canvas FULLSCREEN
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  
  // Draw impossible-to-miss elements
  ctx.fillStyle = '#FF0000'; // Bright red
  ctx.fillRect(0, 0, canvas.width, canvas.height); // Fullscreen red
  
  ctx.fillStyle = '#00FF00'; // Bright green
  ctx.font = 'bold 48px Arial';
  ctx.fillText('VISIBLE!', 50, 100);
  
  console.log('Canvas should be RED with GREEN text');
});
