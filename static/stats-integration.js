// stats_visualization.js
// This file connects the SimulationStatsVisualizer to the socket events

// Create a container for the stats visualization
function initializeStatsVisualization() {
  const statsContainer = document.createElement('div');
  statsContainer.id = 'stats-container';
  statsContainer.style.position = 'absolute';
  statsContainer.style.right = '20px';
  statsContainer.style.top = '80px';
  statsContainer.style.width = '400px';
  statsContainer.style.maxHeight = '80vh';
  statsContainer.style.overflowY = 'auto';
  statsContainer.style.zIndex = '1000';
  statsContainer.style.display = 'none';
  statsContainer.style.boxShadow = '0 0 10px rgba(0,0,0,0.2)';
  statsContainer.style.backgroundColor = '#f8f9fa';
  statsContainer.style.borderRadius = '8px';
  document.body.appendChild(statsContainer);

  // Create a toggle button for the stats panel
  const statsToggleBtn = document.createElement('button');
  statsToggleBtn.innerText = 'Show Stats';
  statsToggleBtn.style.position = 'absolute';
  statsToggleBtn.style.top = '10px';
  statsToggleBtn.style.right = '20px';
  statsToggleBtn.style.zIndex = '1001';
  statsToggleBtn.style.padding = '8px 12px';
  statsToggleBtn.style.backgroundColor = '#4CAF50';
  statsToggleBtn.style.color = 'white';
  statsToggleBtn.style.border = 'none';
  statsToggleBtn.style.borderRadius = '4px';
  statsToggleBtn.style.cursor = 'pointer';
  document.body.appendChild(statsToggleBtn);

  // State tracking
  let statsVisible = false;
  let currentStats = null;

  // Toggle stats panel visibility
  statsToggleBtn.addEventListener('click', () => {
    statsVisible = !statsVisible;
    statsContainer.style.display = statsVisible ? 'block' : 'none';
    statsToggleBtn.innerText = statsVisible ? 'Hide Stats' : 'Show Stats';
    statsToggleBtn.style.backgroundColor = statsVisible ? '#f44336' : '#4CAF50';
    
    // Render current stats if available
    if (statsVisible && currentStats) {
      renderStats(currentStats);
    }
  });

  // Function to render the stats
  function renderStats(stats) {
    if (!stats) return;
    
    const statsSummary = document.createElement('div');
    statsSummary.innerHTML = generateStatsHTML(stats);
    
    // Clear previous content
    statsContainer.innerHTML = '';
    statsContainer.appendChild(statsSummary);
  }

  // Generate HTML for stats display
  function generateStatsHTML(stats) {
    if (!stats) return '<div class="p-4">No data available</div>';
    
    // Format number helper
    const formatNumber = (num) => {
      if (num === undefined || num === null) return 'N/A';
      return typeof num === 'number' ? num.toFixed(2) : num;
    };

    return `
      <div class="p-4 bg-gray-100 rounded-lg">
        <h2 class="text-xl font-bold mb-4">Simulation Statistics</h2>
        
        <!-- Summary Section -->
        <div class="grid grid-cols-2 gap-4 mb-6">
          <div class="bg-white p-3 rounded shadow">
            <div class="font-semibold text-gray-600">Total Distance</div>
            <div class="text-xl">${formatNumber(stats.total_distance)}</div>
          </div>
          <div class="bg-white p-3 rounded shadow">
            <div class="font-semibold text-gray-600">Simulation Time</div>
            <div class="text-xl">${formatNumber(stats.final_time)}</div>
          </div>
          <div class="bg-white p-3 rounded shadow">
            <div class="font-semibold text-gray-600">Light Changes</div>
            <div class="text-xl">${stats.total_light_changes || 0}</div>
          </div>
          <div class="bg-white p-3 rounded shadow">
            <div class="font-semibold text-gray-600">Completed Epochs</div>
            <div class="text-xl">${stats.epochs_completed || 0}</div>
          </div>
        </div>
        
        <!-- Robot Info -->
        <div class="bg-white p-4 rounded shadow mb-4">
          <h3 class="text-lg font-semibold mb-2">Simulation Info</h3>
          <div class="grid grid-cols-2 gap-4">
            <div>
              <div class="font-medium text-gray-600">Number of Robots</div>
              <div>${stats.num_robots || 0}</div>
            </div>
            <div>
              <div class="font-medium text-gray-600">Terminated Robots</div>
              <div>${stats.terminated_robots || 0}</div>
            </div>
            <div>
              <div class="font-medium text-gray-600">Algorithm</div>
              <div>${stats.algorithm || 'N/A'}</div>
            </div>
            <div>
              <div class="font-medium text-gray-600">Seed</div>
              <div>${stats.simulation_config?.seed || 'N/A'}</div>
            </div>
          </div>
        </div>
        
        <!-- Performance Metrics -->
        <div class="bg-white p-4 rounded shadow">
          <h3 class="text-lg font-semibold mb-2">Performance Metrics</h3>
          <div class="grid grid-cols-2 gap-4">
            <div>
              <div class="font-medium text-gray-600">Avg Distance/Robot</div>
              <div>${formatNumber(stats.avg_distance_per_robot)}</div>
            </div>
            <div>
              <div class="font-medium text-gray-600">Avg Light Changes</div>
              <div>${formatNumber(stats.avg_light_changes_per_robot)}</div>
            </div>
            <div>
              <div class="font-medium text-gray-600">Convergence</div>
              <div>${formatNumber((stats.terminated_robots / stats.num_robots) * 100)}%</div>
            </div>
          </div>
        </div>
      </div>
    `;
  }

  return {
    updateStats: function(stats) {
      currentStats = stats;
      if (statsVisible) {
        renderStats(stats);
      }
    },
    clearStats: function() {
      currentStats = null;
      statsContainer.innerHTML = '<div class="p-4">No simulation data available</div>';
    },
    showStats: function() {
      if (!statsVisible) {
        statsToggleBtn.click();
      }
    }
  };
}

// Create and initialize the visualization
const statsVisualizer = initializeStatsVisualization();

// Connect to socket events - to be called after socket is initialized
function connectStatsToSocket(socket) {
  socket.on("simulation_stats", function(data) {
    try {
      const statsData = JSON.parse(data);
      console.log("Received simulation stats:", statsData);
      
      if (statsData && statsData.stats) {
        statsVisualizer.updateStats(statsData.stats);
        
        // Automatically show stats when they arrive
        statsVisualizer.showStats();
      }
    } catch (error) {
      console.error("Error processing stats data:", error);
    }
  });
  
  socket.on("simulation_start", function() {
    // Reset stats on new simulation
    statsVisualizer.clearStats();
  });
  
  socket.on("simulation_end", function() {
    // Show stats when simulation ends
    setTimeout(() => {
      statsVisualizer.showStats();
    }, 500);
  });
}

// Expose the function to connect to socket
window.connectStatsToSocket = connectStatsToSocket;

// This should be used in main.js after socket initialization:
// if (window.connectStatsToSocket) window.connectStatsToSocket(socket);
