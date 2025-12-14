function occMap = createOccupancyMapWithObstacles()
%CREATEOCCUPANCYMAPWITHOBSTACLES Create occupancy map with two square obstacles
%
%   occMap = createOccupancyMapWithObstacles()
%
%   Creates a 20x10 meter map with 0.5m resolution
%   Obstacle 1: 4x4m square centered at (2, 4)
%   Obstacle 2: 4x4m square centered at (14, 4)

    % Map dimensions and resolution
    mapWidth = 20;   % meters
    mapHeight = 40;  % meters
    resolution = 0.5; % meters per cell (2 cells per meter)
    occMatrix = zeros(mapWidth,mapHeight);  % all free (0)

    
    % Create occupancy map
    occMap = occupancyMap(occMatrix, 1/resolution);
    center = [-2,-1];
    occMap.GridOriginInLocal = center;
    
    % Define obstacles (4x4 meter squares)
    % Obstacle 1: center at (2, 4)
    obs1_center = [2, 4];% - center;
    obs1_size = 4;
    obs1_x = obs1_center(1) + [-obs1_size/2, obs1_size/2, obs1_size/2, -obs1_size/2, -obs1_size/2];
    obs1_y = obs1_center(2) + [-obs1_size/2, -obs1_size/2, obs1_size/2, obs1_size/2, -obs1_size/2];
    
    % Obstacle 2: center at (14, 4)
    obs2_center = [14, 4];% - center;
    obs2_size = 4;
    obs2_x = obs2_center(1) + [-obs2_size/2, obs2_size/2, obs2_size/2, -obs2_size/2, -obs2_size/2];
    obs2_y = obs2_center(2) + [-obs2_size/2, -obs2_size/2, obs2_size/2, obs2_size/2, -obs2_size/2];
    
    % Fill obstacles with occupied cells
    % Create grid of points inside each obstacle
    obs1_xmin = obs1_center(1) - obs1_size/2;
    obs1_xmax = obs1_center(1) + obs1_size/2;
    obs1_ymin = obs1_center(2) - obs1_size/2;
    obs1_ymax = obs1_center(2) + obs1_size/2;
    
    obs2_xmin = obs2_center(1) - obs2_size/2;
    obs2_xmax = obs2_center(1) + obs2_size/2;
    obs2_ymin = obs2_center(2) - obs2_size/2;
    obs2_ymax = obs2_center(2) + obs2_size/2;
    
    % Set occupied cells
    [xGrid, yGrid] = meshgrid(0:resolution:mapWidth, 0:resolution:mapHeight);
    
    % Obstacle 1
    mask1 = (xGrid > obs1_xmin) & (xGrid <= obs1_xmax) & ...
            (yGrid > obs1_ymin) & (yGrid <= obs1_ymax);
    
    % Obstacle 2
    mask2 = (xGrid > obs2_xmin) & (xGrid <= obs2_xmax) & ...
            (yGrid > obs2_ymin) & (yGrid <= obs2_ymax);
    
    % Combine masks and set occupancy
    occupiedPoints = [xGrid(mask1 | mask2), yGrid(mask1 | mask2)];
    setOccupancy(occMap, occupiedPoints, ones(size(occupiedPoints, 1), 1));
end
