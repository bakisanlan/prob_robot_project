function [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory()
%GENERATERECTANGLEOBSTACLETRAJECTORY Generate rectangle trajectory with obstacles
%
%   [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory()
%
%   Outputs:
%       X_gt    - [3×N] Ground truth poses [x; y; theta]
%       U_list  - [2×(N-1)] Control inputs [v; w]
%       x0      - [3×1] Initial pose
%       dt      - Time step (seconds)
%       occMap  - occupancyMap object with obstacles

    % Generate base rectangle trajectory
    [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
    
    % Create occupancy map with obstacles
    occMap = createOccupancyMapWithObstacles();
end
