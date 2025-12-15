classdef SensorsCallbacks < handle
%SENSORSCALLBACKS Handles all callbacks for the SENSORS panel (2D Lidar)
%
%   This class encapsulates all callback functions related to the SENSORS panel,
%   including lidar configuration, scan creation, and visualization.
%
%   Usage:
%       callbacks = SensorsCallbacks(ax, handles, fig);
%       callbacks.setupCallbacks();

    properties
        ax              % Main axes handle
        handles         % Struct containing all SENSORS panel UI handles
        fig             % Parent figure handle
        lidarScan       % Current lidarScan object
        lidarPlotHandles % Handles to lidar visualization objects
        sensorState     % Sensor state structure
        landmarks       % Array of landmarks [id, x, y]
        landmarkPlots   % Handles to landmark plot markers
        autoUpdateEnabled % Flag for auto-updating LIDAR during simulation
        measurementPlotHandles % Handles to EKF measurement visualization (cyan rays)
    end
    
    methods
        function obj = SensorsCallbacks(ax, handles, fig)
            %SENSORSCALLBACKS Constructor
            obj.ax = ax;
            obj.handles = handles;
            obj.fig = fig;
            obj.lidarScan = [];
            obj.lidarPlotHandles = struct('rays', [], 'noisyRays', [], 'endpoints', []);
            obj.landmarks = [];  % Empty Nx3 matrix [id, x, y]
            obj.landmarkPlots = [];  % Handles to landmark markers
            obj.autoUpdateEnabled = false;  % Auto-update disabled by default
            obj.measurementPlotHandles = [];  % Handles to measurement visualization
            
            % Initialize sensor state
            obj.sensorState = struct();
            obj.sensorState.maxRange = 10;
            obj.sensorState.fov = 90;
            obj.sensorState.resolution = 2;
            obj.sensorState.rangeNoise = 0.1;
            obj.sensorState.angularNoise = 0.5;
            obj.sensorState.currentPose = [0; 0; 0];
        end
        
        function setupCallbacks(obj)
            %SETUPCALLBACKS Attach callbacks to UI components
            
            % Parameter change callbacks
            set(obj.handles.editRange, 'Callback', @(~,~) obj.updateScanInfo());
            set(obj.handles.editFOV, 'Callback', @(~,~) obj.updateScanInfo());
            set(obj.handles.editResolution, 'Callback', @(~,~) obj.updateScanInfo());
            set(obj.handles.editRangeNoise, 'Callback', @(~,~) obj.updateScanInfo());
            set(obj.handles.editAngularNoise, 'Callback', @(~,~) obj.updateScanInfo());
            
            % Button callbacks
            set(obj.handles.btnCreateScan, 'Callback', @(~,~) obj.createLidarScan());
            set(obj.handles.btnClearScan, 'Callback', @(~,~) obj.clearScan());
            
            % Visualization checkboxes
            set(obj.handles.chkShowLidar, 'Callback', @(~,~) obj.updateVisualization());
            set(obj.handles.chkShowNoisyRays, 'Callback', @(~,~) obj.updateVisualization());
            
            % Landmark callbacks
            set(obj.handles.btnAddManual, 'Callback', @(~,~) obj.addLandmarkManual());
            set(obj.handles.btnAddByClick, 'Callback', @(~,~) obj.addLandmarkByClick());
            set(obj.handles.btnRemoveLandmark, 'Callback', @(~,~) obj.removeLandmark());
            set(obj.handles.btnClearLandmarks, 'Callback', @(~,~) obj.clearLandmarks());
            
            % Initialize scan info display
            obj.updateScanInfo();
        end
        
        function updateScanInfo(obj)
            %UPDATESCANINFO Update the scan information display
            
            % Get parameters
            maxRange = str2double(get(obj.handles.editRange, 'String'));
            fov = str2double(get(obj.handles.editFOV, 'String'));
            resolution = str2double(get(obj.handles.editResolution, 'String'));
            rangeNoise = str2double(get(obj.handles.editRangeNoise, 'String'));
            angularNoise = str2double(get(obj.handles.editAngularNoise, 'String'));
            
            % Validate inputs
            if isnan(maxRange) || maxRange <= 0
                maxRange = 10;
                set(obj.handles.editRange, 'String', '10');
            end
            if isnan(fov) || fov <= 0 || fov > 360
                fov = 90;
                set(obj.handles.editFOV, 'String', '90');
            end
            if isnan(resolution) || resolution <= 0 || resolution > fov
                resolution = 2;
                set(obj.handles.editResolution, 'String', '2');
            end
            if isnan(rangeNoise) || rangeNoise < 0
                rangeNoise = 0.1;
                set(obj.handles.editRangeNoise, 'String', '0.1');
            end
            if isnan(angularNoise) || angularNoise < 0
                angularNoise = 0.5;
                set(obj.handles.editAngularNoise, 'String', '0.5');
            end
            
            % Store in state
            obj.sensorState.maxRange = maxRange;
            obj.sensorState.fov = fov;
            obj.sensorState.resolution = resolution;
            obj.sensorState.rangeNoise = rangeNoise;
            obj.sensorState.angularNoise = angularNoise;
            
            % Calculate angles
            halfFOV = fov / 2;
            angles = -deg2rad(halfFOV):deg2rad(resolution):deg2rad(halfFOV);
            numRays = length(angles);
            
            % Update display
            set(obj.handles.txtNumRays, 'String', sprintf('Rays: %d', numRays));
            set(obj.handles.txtAngleRange, 'String', sprintf('Angles: -%.1f° to %.1f°', halfFOV, halfFOV));
        end
        
        function createLidarScan(obj)
            %CREATELIDARSCAN Create a lidarScan object with current parameters
            
            try
                % Get parameters
                maxRange = obj.sensorState.maxRange;
                fov = obj.sensorState.fov;
                resolution = obj.sensorState.resolution;
                
                % Calculate angles (symmetric around 0)
                halfFOV = fov / 2;
                angles = -deg2rad(halfFOV):deg2rad(resolution):deg2rad(halfFOV);
                
                % Get robot pose and occupancy map from MODEL panel
                robotPose = [0; 0; 0];  % Default pose
                occMap = [];
                
                if isappdata(obj.fig, 'modelCallbacks')
                    modelCallbacks = getappdata(obj.fig, 'modelCallbacks');
                    if ~isempty(modelCallbacks.simState.X_gt)
                        % Get current pose from ground truth trajectory
                        if modelCallbacks.simState.currentStep > 0
                            robotPose = modelCallbacks.simState.X_gt(:, modelCallbacks.simState.currentStep + 1);
                        else
                            robotPose = modelCallbacks.simState.x0;
                        end
                    end
                    occMap = modelCallbacks.simState.occMap;
                end
                
                obj.sensorState.currentPose = robotPose;
                
                % Perform raycasting if occupancy map exists
                if ~isempty(occMap)
                    ranges = obj.raycastScan(robotPose, angles, maxRange, occMap);
                else
                    % No obstacles, all at max range
                    ranges = maxRange * ones(1, length(angles));
                end
                
                % Create lidarScan object
                obj.lidarScan = lidarScan(ranges, angles);
                
                % Update status
                obj.setStatus(sprintf('Scan created: %d rays, FOV=%.1f°, Pose=(%.2f,%.2f,%.1f°)', ...
                    length(angles), fov, robotPose(1), robotPose(2), rad2deg(robotPose(3))), [0, 0.5, 0]);
                
                % Enable visualization checkbox and show scan
                set(obj.handles.chkShowLidar, 'Value', 1);
                obj.visualizeScan(robotPose);
                
                % Enable auto-update for live simulation
                obj.autoUpdateEnabled = true;
                
            catch ME
                obj.setStatus(sprintf('Error: %s', ME.message), [0.8, 0, 0]);
            end
        end
        
        function visualizeScan(obj, pose)
            %VISUALIZESCAN Visualize the lidar scan at a given pose
            %   pose: [x; y; theta] robot pose
            
            if isempty(obj.lidarScan)
                obj.setStatus('No scan created. Click "Create Scan" first.', [0.8, 0.5, 0]);
                return;
            end
            
            % Clear previous visualization
            obj.clearVisualization();
            
            % Get scan data
            ranges = obj.lidarScan.Ranges;
            angles = obj.lidarScan.Angles;
            
            % Robot position and orientation
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Transform angles to world frame
            worldAngles = angles + theta;
            
            % Calculate end points
            endX = x + ranges .* cos(worldAngles);
            endY = y + ranges .* sin(worldAngles);
            
            % Plot rays
            hold(obj.ax, 'on');
            
            if get(obj.handles.chkShowLidar, 'Value')
                % Plot each ray (excluded from legend)
                for i = 1:length(ranges)
                    obj.lidarPlotHandles.rays(i) = plot(obj.ax, ...
                        [x, endX(i)], [y, endY(i)], ...
                        'b-', 'LineWidth', 0.5, 'Color', [0, 0.5, 1, 0.3], ...
                        'HandleVisibility', 'off');
                end
                
                % Plot endpoints (shown in legend)
                obj.lidarPlotHandles.endpoints = plot(obj.ax, endX, endY, ...
                    'b.', 'MarkerSize', 6, 'DisplayName', 'Lidar Points');
            end
            
            % Show noisy readings if enabled
            if get(obj.handles.chkShowNoisyRays, 'Value')
                noisyScan = obj.addNoiseToScan();
                noisyRanges = noisyScan.Ranges;
                noisyAngles = noisyScan.Angles + theta;
                
                noisyEndX = x + noisyRanges .* cos(noisyAngles);
                noisyEndY = y + noisyRanges .* sin(noisyAngles);
                
                obj.lidarPlotHandles.noisyRays = plot(obj.ax, noisyEndX, noisyEndY, ...
                    'r.', 'MarkerSize', 8, 'DisplayName', 'Noisy Readings');
            end
            
            drawnow;
        end
        
        function ranges = raycastScan(obj, pose, angles, maxRange, occMap)
            %RAYCASTSCAN Perform raycasting to detect obstacles using built-in raycast
            %   pose: [x; y; theta] robot pose
            %   angles: array of sensor angles relative to robot
            %   maxRange: maximum sensor range
            %   occMap: occupancyMap object
            %   Returns: array of detected ranges
            
            % Use MATLAB's built-in raycast function
            % raycast expects pose as [x, y, theta] (row vector)
            poseRow = [pose(1), pose(2), pose(3)];
            
            % raycast returns ranges for each angle
            ranges = raycast(occMap, poseRow, angles, maxRange);
        end
        
        function noisyScan = addNoiseToScan(obj)
            %ADDNOISETOSCAN Add Gaussian noise to the current scan
            
            if isempty(obj.lidarScan)
                noisyScan = [];
                return;
            end
            
            % Get noise parameters
            rangeNoise = str2double(get(obj.handles.editRangeNoise, 'String'));
            angularNoise = str2double(get(obj.handles.editAngularNoise, 'String'));
            
            if isnan(rangeNoise), rangeNoise = 0.1; end
            if isnan(angularNoise), angularNoise = 0.5; end
            
            % Get original scan data
            ranges = obj.lidarScan.Ranges;
            angles = obj.lidarScan.Angles;
            
            % Add Gaussian noise
            noisyRanges = ranges + rangeNoise * randn(size(ranges));
            noisyAngles = angles + deg2rad(angularNoise) * randn(size(angles));
            
            % Ensure ranges are positive
            noisyRanges = max(noisyRanges, 0.01);
            
            % Create noisy scan
            noisyScan = lidarScan(noisyRanges, noisyAngles);
        end
        
        function clearVisualization(obj)
            %CLEARVISUALIZATION Remove lidar visualization from axes
            
            % Delete ray lines
            if isfield(obj.lidarPlotHandles, 'rays') && ~isempty(obj.lidarPlotHandles.rays)
                try
                    validRays = isgraphics(obj.lidarPlotHandles.rays);
                    delete(obj.lidarPlotHandles.rays(validRays));
                catch
                    % If error, just clear the handles
                end
                obj.lidarPlotHandles.rays = [];
            end
            
            % Delete endpoints
            if isfield(obj.lidarPlotHandles, 'endpoints') && ~isempty(obj.lidarPlotHandles.endpoints)
                try
                    if isgraphics(obj.lidarPlotHandles.endpoints)
                        delete(obj.lidarPlotHandles.endpoints);
                    end
                catch
                    % If error, just clear the handle
                end
                obj.lidarPlotHandles.endpoints = [];
            end
            
            % Delete noisy rays
            if isfield(obj.lidarPlotHandles, 'noisyRays') && ~isempty(obj.lidarPlotHandles.noisyRays)
                try
                    if isgraphics(obj.lidarPlotHandles.noisyRays)
                        delete(obj.lidarPlotHandles.noisyRays);
                    end
                catch
                    % If error, just clear the handle
                end
                obj.lidarPlotHandles.noisyRays = [];
            end
        end
        
        function clearScan(obj)
            %CLEARSCAN Clear the current scan and visualization
            
            obj.clearVisualization();
            obj.lidarScan = [];
            obj.autoUpdateEnabled = false;
            obj.setStatus('Scan cleared.', [0.5, 0.5, 0.5]);
        end
        
        function testAtCurrentPose(obj)
            %TESTATCURRENTPOSE Test lidar at robot's current pose with raycasting
            
            % Always recreate scan to update with current robot pose
            obj.createLidarScan();
            
            obj.setStatus(sprintf('Scan at pose (%.2f, %.2f, %.1f°)', ...
                obj.sensorState.currentPose(1), obj.sensorState.currentPose(2), ...
                rad2deg(obj.sensorState.currentPose(3))), [0, 0.5, 0]);
        end
        
        function updateScanAtCurrentPose(obj, robotPose, currentStep)
            %UPDATESCANATCURRENTPOSE Update LIDAR scan at current robot pose from MODEL panel
            %   Called automatically during live simulation if auto-update is enabled
            %   
            %   Inputs (optional):
            %       robotPose - [3x1] current robot pose [x; y; theta]
            %       currentStep - current simulation step number
            
            if ~obj.autoUpdateEnabled
                return;
            end
            
            try
                % If pose not provided, get from MODEL panel
                if nargin < 2 || isempty(robotPose)
                    % Get current robot pose from MODEL panel
                    if ~isappdata(obj.fig, 'modelCallbacks')
                        return;
                    end
                    
                    modelCallbacks = getappdata(obj.fig, 'modelCallbacks');
                    if isempty(modelCallbacks.simState.X_gt)
                        return;
                    end
                    
                    % Get current pose
                    if modelCallbacks.simState.currentStep > 0
                        robotPose = modelCallbacks.simState.X_gt(:, modelCallbacks.simState.currentStep + 1);
                    else
                        robotPose = modelCallbacks.simState.x0;
                    end
                end
                
                obj.sensorState.currentPose = robotPose;
                
                % Get occupancy map from MODEL panel (if available)
                occMap = [];
                if isappdata(obj.fig, 'modelCallbacks')
                    modelCallbacks = getappdata(obj.fig, 'modelCallbacks');
                    occMap = modelCallbacks.simState.occMap;
                end
                
                % Get parameters
                maxRange = obj.sensorState.maxRange;
                fov = obj.sensorState.fov;
                resolution = obj.sensorState.resolution;
                
                % Calculate angles
                halfFOV = fov / 2;
                angles = -deg2rad(halfFOV):deg2rad(resolution):deg2rad(halfFOV);
                
                % Perform raycasting
                if ~isempty(occMap)
                    ranges = obj.raycastScan(robotPose, angles, maxRange, occMap);
                else
                    ranges = maxRange * ones(1, length(angles));
                end
                
                % Update lidarScan object
                obj.lidarScan = lidarScan(ranges, angles);
                
                % Update visualization
                obj.visualizeScan(robotPose);
                
            catch
                % Silently fail to avoid disrupting simulation
            end
        end
        
        function updateVisualization(obj)
            %UPDATEVISUALIZATION Update visualization based on checkbox states
            
            if ~isempty(obj.lidarScan)
                obj.visualizeScan(obj.sensorState.currentPose);
            end
        end
        
        function setStatus(obj, message, color)
            %SETSTATUS Update status display
            set(obj.handles.txtStatus, 'String', ['Status: ', message]);
            set(obj.handles.txtStatus, 'ForegroundColor', color);
        end
        
        function scan = getLidarScan(obj)
            %GETLIDARSCAN Get the current lidarScan object
            scan = obj.lidarScan;
        end
        
        function scan = getNoisyLidarScan(obj)
            %GETNOISYLIDARSCAN Get a noisy version of the current scan
            scan = obj.addNoiseToScan();
        end
        
        function setPose(obj, pose)
            %SETPOSE Set the current robot pose for visualization
            obj.sensorState.currentPose = pose;
            
            % Store in figure appdata for cross-panel access
            setappdata(obj.fig, 'currentPose', pose);
        end
        
        %% Landmark Management Methods
        
        function addLandmarkManual(obj)
            %ADDLANDMARKMANUAL Add landmark using manual X, Y entry
            
            % Get X and Y from edit fields
            xStr = get(obj.handles.editLandmarkX, 'String');
            yStr = get(obj.handles.editLandmarkY, 'String');
            
            x = str2double(xStr);
            y = str2double(yStr);
            
            if isnan(x) || isnan(y)
                obj.setStatus('Invalid X or Y value', [0.8, 0, 0]);
                return;
            end
            
            obj.addLandmark(x, y);
        end
        
        function addLandmarkByClick(obj)
            %ADDLANDMARKBYCLICK Add landmark by clicking on the plot
            
            obj.setStatus('Click on the plot to add landmark...', [0, 0, 0.8]);
            
            % Get user input from plot
            try
                [x, y] = ginput(1);
                
                if ~isempty(x) && ~isempty(y)
                    obj.addLandmark(x, y);
                else
                    obj.setStatus('Landmark placement cancelled', [0.5, 0.5, 0.5]);
                end
            catch
                obj.setStatus('Error getting click input', [0.8, 0, 0]);
            end
        end
        
        function addLandmark(obj, x, y)
            %ADDLANDMARK Add a landmark at position (x, y)
            
            % Generate new ID
            if isempty(obj.landmarks)
                newID = 1;
            else
                newID = max(obj.landmarks(:, 1)) + 1;
            end
            
            % Add to landmarks array
            obj.landmarks = [obj.landmarks; newID, x, y];
            
            % Update list display
            obj.updateLandmarkList();
            
            % Visualize landmark
            obj.visualizeLandmarks();
            
            obj.setStatus(sprintf('Landmark %d added at (%.2f, %.2f)', newID, x, y), [0, 0.5, 0]);
        end
        
        function removeLandmark(obj)
            %REMOVELANDMARK Remove selected landmark from list
            
            selectedIdx = get(obj.handles.listLandmarks, 'Value');
            
            if isempty(obj.landmarks) || selectedIdx < 1 || selectedIdx > size(obj.landmarks, 1)
                obj.setStatus('No landmark selected', [0.8, 0.5, 0]);
                return;
            end
            
            removedID = obj.landmarks(selectedIdx, 1);
            obj.landmarks(selectedIdx, :) = [];
            
            % Update list and visualization
            obj.updateLandmarkList();
            obj.visualizeLandmarks();
            
            % Reset selection
            if ~isempty(obj.landmarks)
                newIdx = min(selectedIdx, size(obj.landmarks, 1));
                set(obj.handles.listLandmarks, 'Value', newIdx);
            end
            
            obj.setStatus(sprintf('Landmark %d removed', removedID), [0.5, 0.5, 0]);
        end
        
        function clearLandmarks(obj)
            %CLEARLANDMARKS Clear all landmarks
            
            obj.landmarks = [];
            obj.updateLandmarkList();
            obj.clearLandmarkVisualization();
            
            obj.setStatus('All landmarks cleared', [0.5, 0.5, 0]);
        end
        
        function updateLandmarkList(obj)
            %UPDATELANDMARKLIST Update the landmark listbox display
            
            if isempty(obj.landmarks)
                set(obj.handles.listLandmarks, 'String', {}, 'Value', 1);
            else
                listStr = cell(size(obj.landmarks, 1), 1);
                for i = 1:size(obj.landmarks, 1)
                    listStr{i} = sprintf('ID:%d  X:%.2f  Y:%.2f', ...
                        obj.landmarks(i, 1), obj.landmarks(i, 2), obj.landmarks(i, 3));
                end
                set(obj.handles.listLandmarks, 'String', listStr);
                
                % Ensure valid selection
                currentVal = get(obj.handles.listLandmarks, 'Value');
                if isempty(currentVal) || currentVal(1) < 1 || currentVal(1) > size(obj.landmarks, 1)
                    set(obj.handles.listLandmarks, 'Value', 1);
                end
            end
        end
        
        function visualizeLandmarks(obj)
            %VISUALIZELANDMARKS Visualize all landmarks on the plot
            
            % Clear previous visualization
            obj.clearLandmarkVisualization();
            
            if isempty(obj.landmarks)
                return;
            end
            
            hold(obj.ax, 'on');
            
            % Plot each landmark
            for i = 1:size(obj.landmarks, 1)
                id = obj.landmarks(i, 1);
                x = obj.landmarks(i, 2);
                y = obj.landmarks(i, 3);
                
                % Plot marker (purple color)
                h = plot(obj.ax, x, y, '*', 'MarkerSize', 12, 'LineWidth', 2, ...
                    'Color', [0.5, 0, 0.5], ...
                    'DisplayName', sprintf('Landmark %d', id));
                obj.landmarkPlots = [obj.landmarkPlots; h];
                
                % Add text label (purple, closer to marker)
                hText = text(obj.ax, x, y+0.05, sprintf('%d', id), ...
                    'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.5, 0, 0.5], ...
                    'HorizontalAlignment', 'center');
                obj.landmarkPlots = [obj.landmarkPlots; hText];
            end
            
            drawnow;
        end
        
        function clearLandmarkVisualization(obj)
            %CLEARLANDMARKVISUALIZATION Remove landmark markers from plot
            
            if ~isempty(obj.landmarkPlots)
                try
                    validPlots = isgraphics(obj.landmarkPlots);
                    delete(obj.landmarkPlots(validPlots));
                catch
                    % If error, just clear
                end
                obj.landmarkPlots = [];
            end
        end
        
        function visualizeLandmarkMeasurements(obj, robotPose, detectedLandmarks, measurements)
            %VISUALIZELANDMARKMEASUREMENTS Visualize landmark measurements from EKF
            %   Inputs:
            %       robotPose - [3x1] current robot pose
            %       detectedLandmarks - [Kx3] detected landmarks [id, x, y]
            %       measurements - struct with .ranges, .bearings (with noise)
            
            % Clear previous measurement visualization
            obj.clearMeasurementVisualization();
            
            if isempty(detectedLandmarks) || isempty(measurements)
                return;
            end
            
            % Extract robot position and orientation
            xr = robotPose(1);
            yr = robotPose(2);
            theta = robotPose(3);
            
            hold(obj.ax, 'on');
            
            % Plot measurement rays from robot to detected landmarks
            for i = 1:size(detectedLandmarks, 1)
                % Use noisy measurements
                range = measurements.ranges(i);
                bearing = measurements.bearings(i);
                
                % Compute endpoint using noisy measurement
                worldAngle = theta + bearing;
                endX = xr + range * cos(worldAngle);
                endY = yr + range * sin(worldAngle);
                
                % Plot ray (cyan color for EKF measurements)
                h1 = plot(obj.ax, [xr, endX], [yr, endY], 'c-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
                obj.measurementPlotHandles = [obj.measurementPlotHandles; h1];
                
                % Plot endpoint (cyan dot)
                h2 = plot(obj.ax, endX, endY, 'co', 'MarkerSize', 8, 'LineWidth', 2, 'HandleVisibility', 'off');
                obj.measurementPlotHandles = [obj.measurementPlotHandles; h2];
            end
            
            drawnow;
        end
        
        function clearMeasurementVisualization(obj)
            %CLEARMEASUREMENTVISUALIZATION Remove measurement visualization from plot
            
            if ~isempty(obj.measurementPlotHandles)
                try
                    validPlots = isgraphics(obj.measurementPlotHandles);
                    delete(obj.measurementPlotHandles(validPlots));
                catch
                    % If error, just clear
                end
                obj.measurementPlotHandles = [];
            end
        end
    end
end
