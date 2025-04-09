// data_processing_worker.js
// Web Worker for handling data processing to prevent UI freezing

// Cache for optimized calculations
const calculationCache = new Map();

// Process incoming messages
self.onmessage = function(e) {
  const { type, data } = e.data;
  
  switch(type) {
    case 'process_batch':
      const processedData = processBatch(data);
      self.postMessage({
        type: 'batch_processed',
        data: processedData
      });
      break;
      
    case 'calculate_stats':
      const stats = calculateStats(data);
      self.postMessage({
        type: 'stats_calculated',
        data: stats
      });
      break;
      
    case 'clear_cache':
      calculationCache.clear();
      self.postMessage({
        type: 'cache_cleared'
      });
      break;
      
    case 'detect_movement':
      const movementData = detectSignificantMovement(data);
      self.postMessage({
        type: 'movement_detected',
        data: movementData
      });
      break;
  }
};

// Process a batch of snapshots efficiently
function processBatch(snapshots) {
  if (!snapshots || !Array.isArray(snapshots)) return [];
  
  // Calculate movement metrics
  const processedSnapshots = snapshots.map(snapshot => {
    const robotMovements = calculateRobotMovements(snapshot);
    return {
      ...snapshot,
      robotMovements,
      needsFullRedraw: robotMovements.significantMovement
    };
  });
  
  return processedSnapshots;
}

// Analyze if there's significant movement in a snapshot
function calculateRobotMovements(snapshot) {
  if (!snapshot || !snapshot.snapshot) {
    return { significantMovement: false, movements: [] };
  }
  
  const movements = [];
  let significantMovement = false;
  const movementThreshold = 0.01;
  
  // Check for robot position changes
  Object.entries(snapshot.snapshot).forEach(([id, details]) => {
    const position = details.pos;
    const cacheKey = `${id}-prev-pos`;
    const prevPos = calculationCache.get(cacheKey);
    
    if (prevPos) {
      const distance = calculateDistance(
        position.x, position.y, 
        prevPos.x, prevPos.y
      );
      
      movements.push({
        id,
        distance,
        significant: distance > movementThreshold
      });
      
      if (distance > movementThreshold) {
        significantMovement = true;
      }
    }
    
    // Update cache
    calculationCache.set(cacheKey, position);
  });
  
  return {
    significantMovement,
    movements
  };
}

// Calculate distance between two points
function calculateDistance(x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  return Math.sqrt(dx * dx + dy * dy);
}

// Calculate extended statistics
function calculateStats(data) {
  if (!data || !data.stats) return null;
  
  const { stats } = data;
  const extendedStats = { ...stats };
  
  // Calculate additional metrics
  if (stats.robot_details && stats.robot_details.length > 0) {
    // Calculate efficiency (distance traveled / time)
    extendedStats.efficiency = stats.total_distance / Math.max(1, stats.final_time);
    
    // Calculate variance in robot movement
    const distances = stats.robot_details.map(r => r.distance_travelled);
    extendedStats.distanceVariance = calculateVariance(distances);
    
    // Calculate max/min distances
    extendedStats.maxDistance = Math.max(...distances);
    extendedStats.minDistance = Math.min(...distances);
    
    // Calculate color distribution
    const colorCounts = {};
    stats.robot_details.forEach(robot => {
      colorCounts[robot.color] = (colorCounts[robot.color] || 0) + 1;
    });
    extendedStats.colorDistribution = colorCounts;
    
    // Calculate termination rate
    const terminatedCount = stats.robot_details.filter(r => r.terminated).length;
    extendedStats.terminationRate = terminatedCount / stats.robot_details.length;
    
    // Calculate average position
    let sumX = 0, sumY = 0;
    stats.robot_details.forEach(robot => {
      sumX += robot.final_position.x;
      sumY += robot.final_position.y;
    });
    extendedStats.averagePosition = {
      x: sumX / stats.robot_details.length,
      y: sumY / stats.robot_details.length
    };
  }
  
  return extendedStats;
}

// Helper function to calculate variance
function calculateVariance(values) {
  if (!values || values.length === 0) return 0;
  
  const mean = values.reduce((sum, val) => sum + val, 0) / values.length;
  const squaredDiffs = values.map(val => (val - mean) ** 2);
  return squaredDiffs.reduce((sum, val) => sum + val, 0) / values.length;
}

// Analyze robot movement patterns over time
function detectSignificantMovement(snapshotHistory) {
  if (!snapshotHistory || !Array.isArray(snapshotHistory) || snapshotHistory.length < 2) {
    return { hasSignificantMovement: false };
  }
  
  const movementTracker = new Map();
  let significantMovementDetected = false;
  let movementDirection = { x: 0, y: 0 };
  
  // Calculate movement vectors between consecutive snapshots
  for (let i = 1; i < snapshotHistory.length; i++) {
    const prevSnapshot = snapshotHistory[i-1].snapshot;
    const currSnapshot = snapshotHistory[i].snapshot;
    
    // For each robot, calculate movement
    Object.keys(currSnapshot).forEach(id => {
      if (prevSnapshot[id]) {
        const prevPos = prevSnapshot[id].pos;
        const currPos = currSnapshot[id].pos;
        
        const moveX = currPos.x - prevPos.x;
        const moveY = currPos.y - prevPos.y;
        const moveDist = Math.sqrt(moveX * moveX + moveY * moveY);
        
        if (moveDist > 0.1) { // Threshold for significant movement
          significantMovementDetected = true;
          movementDirection.x += moveX;
          movementDirection.y += moveY;
          
          // Track this robot's movement
          if (!movementTracker.has(id)) {
            movementTracker.set(id, { 
              totalDistance: 0, 
              movements: [] 
            });
          }
          
          const robotTracker = movementTracker.get(id);
          robotTracker.totalDistance += moveDist;
          robotTracker.movements.push({
            time: snapshotHistory[i].time,
            distance: moveDist,
            direction: { x: moveX, y: moveY }
          });
        }
      }
    });
  }
  
  // Normalize movement direction vector
  const dirMagnitude = Math.sqrt(
    movementDirection.x * movementDirection.x + 
    movementDirection.y * movementDirection.y
  );
  
  if (dirMagnitude > 0) {
    movementDirection.x /= dirMagnitude;
    movementDirection.y /= dirMagnitude;
  }
  
  return {
    hasSignificantMovement: significantMovementDetected,
    movementDirection,
    robotMovements: Array.from(movementTracker.entries()).map(([id, data]) => ({
      robotId: id,
      totalDistance: data.totalDistance,
      movementCount: data.movements.length,
      averageMovement: data.totalDistance / Math.max(1, data.movements.length)
    }))
  };
}
