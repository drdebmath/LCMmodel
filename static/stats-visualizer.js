// SimulationStatsVisualizer.jsx
import React, { useState, useEffect } from 'react';

/**
 * A React component that visualizes simulation statistics.
 * This component should be placed in /static/ directory.
 */
const SimulationStatsVisualizer = ({ stats }) => {
  const [timeSeriesData, setTimeSeriesData] = useState([]);
  const [robotData, setRobotData] = useState([]);
  const [showDetails, setShowDetails] = useState(false);

  // Process stats data when it changes
  useEffect(() => {
    if (!stats) return;
    
    // Convert robot details to chart-friendly format
    if (stats.robot_details) {
      const robotChartData = stats.robot_details.map(robot => ({
        id: `Robot ${robot.id}`,
        distance: robot.distance_travelled,
        lightChanges: robot.light_changes,
        color: robot.color,
        terminated: robot.terminated ? 'Yes' : 'No'
      }));
      setRobotData(robotChartData);
    }
    
    // Add to time series if we're receiving incremental updates
    if (stats.final_time && stats.total_distance) {
      setTimeSeriesData(prev => {
        // Check if we already have this timestamp
        const exists = prev.some(item => Math.abs(item.time - stats.final_time) < 0.001);
        if (!exists) {
          return [...prev, {
            time: stats.final_time,
            totalDistance: stats.total_distance,
            lightChanges: stats.total_light_changes || 0
          }];
        }
        return prev;
      });
    }
  }, [stats]);

  if (!stats) {
    return <div className="p-4 text-center">Waiting for simulation data...</div>;
  }

  // Format numbers for display
  const formatNumber = (num) => {
    if (num === undefined || num === null) return 'N/A';
    return typeof num === 'number' ? num.toFixed(2) : num;
  };

  return (
    <div className="p-4 bg-gray-100 rounded-lg text-sm">
      <h2 className="text-xl font-bold mb-4">Simulation Statistics</h2>
      
      {/* Summary Section */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
        <div className="bg-white p-3 rounded shadow">
          <div className="font-semibold text-gray-600">Total Distance</div>
          <div className="text-xl">{formatNumber(stats.total_distance)}</div>
        </div>
        <div className="bg-white p-3 rounded shadow">
          <div className="font-semibold text-gray-600">Simulation Time</div>
          <div className="text-xl">{formatNumber(stats.final_time)}</div>
        </div>
        <div className="bg-white p-3 rounded shadow">
          <div className="font-semibold text-gray-600">Light Changes</div>
          <div className="text-xl">{stats.total_light_changes || 0}</div>
        </div>
        <div className="bg-white p-3 rounded shadow">
          <div className="font-semibold text-gray-600">Completed Epochs</div>
          <div className="text-xl">{stats.epochs_completed || 0}</div>
        </div>
      </div>

      {/* Additional Stats */}
      <div className="bg-white p-4 rounded shadow mb-6">
        <h3 className="text-lg font-semibold mb-2">Performance Metrics</h3>
        <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
          <div>
            <div className="font-medium text-gray-600">Avg Distance/Robot</div>
            <div>{formatNumber(stats.avg_distance_per_robot)}</div>
          </div>
          <div>
            <div className="font-medium text-gray-600">Avg Light Changes</div>
            <div>{formatNumber(stats.avg_light_changes_per_robot)}</div>
          </div>
          <div>
            <div className="font-medium text-gray-600">Terminated Robots</div>
            <div>{stats.terminated_robots || 0}/{stats.num_robots || 0}</div>
          </div>
          <div>
            <div className="font-medium text-gray-600">Convergence</div>
            <div>
              {stats.convergence_percentage ? 
                `${formatNumber(stats.convergence_percentage)}%` : 
                `${(((stats.terminated_robots || 0) / (stats.num_robots || 1)) * 100).toFixed(1)}%`}
            </div>
          </div>
          <div>
            <div className="font-medium text-gray-600">Avg Speed</div>
            <div>{formatNumber(stats.avg_speed)}</div>
          </div>
          <div>
            <div className="font-medium text-gray-600">Algorithm</div>
            <div>{stats.algorithm || 'N/A'}</div>
          </div>
        </div>
      </div>
      
      {/* Toggle for details */}
      <button 
        className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded mb-4"
        onClick={() => setShowDetails(!showDetails)}
      >
        {showDetails ? 'Hide Details' : 'Show Robot Details'}
      </button>
      
      {/* Robot Details Table (shown when expanded) */}
      {showDetails && robotData.length > 0 && (
        <div className="bg-white p-4 rounded shadow mb-6 overflow-x-auto">
          <h3 className="text-lg font-semibold mb-2">Robot Details</h3>
          <table className="min-w-full border">
            <thead className="bg-gray-200">
              <tr>
                <th className="border px-2 py-1">ID</th>
                <th className="border px-2 py-1">Distance</th>
                <th className="border px-2 py-1">Light Changes</th>
                <th className="border px-2 py-1">Color</th>
                <th className="border px-2 py-1">Terminated</th>
              </tr>
            </thead>
            <tbody>
              {robotData.map((robot, index) => (
                <tr key={index} className={index % 2 === 0 ? 'bg-gray-50' : ''}>
                  <td className="border px-2 py-1">{robot.id}</td>
                  <td className="border px-2 py-1">{formatNumber(robot.distance)}</td>
                  <td className="border px-2 py-1">{robot.lightChanges}</td>
                  <td className="border px-2 py-1">
                    <div className="flex items-center">
                      <div 
                        className="w-4 h-4 mr-2 rounded-full" 
                        style={{ 
                          backgroundColor: robot.color.startsWith('#') ? 
                            robot.color : 
                            (['red', 'blue', 'green', 'white', 'black'].includes(robot.color) ? 
                              robot.color : '#000000')
                        }}
                      />
                      {robot.color}
                    </div>
                  </td>
                  <td className="border px-2 py-1">{robot.terminated}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
      
      {/* Notes about the statistics */}
      <div className="bg-gray-200 p-3 rounded text-xs text-gray-700 mt-4">
        <p>
          <strong>Note:</strong> Statistics are calculated at the end of the simulation. 
          Performance metrics may vary based on algorithm selection and robot configuration.
        </p>
      </div>
    </div>
  );
};

export default SimulationStatsVisualizer;
