classdef GNCCallbacks < handle
%GNCCALLBACKS Handles all callbacks for the GNC panel (EKF Localization)
%
%   This class encapsulates all callback functions related to the GNC panel,
%   including EKF initialization, execution, and visualization.
%
%   Usage:
%       callbacks = GNCCallbacks(ax, handles, fig);
%       callbacks.setupCallbacks();

    properties
        ax              % Main axes handle
        handles         % Struct containing all GNC panel UI handles
        fig             % Parent figure handle
        ekfState        % EKF state structure
        plotHandles     % Handles to visualization objects
        ekf             % EKFLocalizer object
    end
    
    methods
        function obj = GNCCallbacks(ax, handles, fig)
            %GNCCALLBACKS Constructor
            obj.ax = ax;
            obj.handles = handles;
            obj.fig = fig;
            
            % Initialize EKF state
            obj.ekfState = struct();
            obj.ekfState.isRunning = false;
            obj.ekfState.isPaused = false;
            obj.ekfState.currentStep = 0;
            obj.ekfState.mu = [];       % State estimate [3×1]
            obj.ekfState.Sigma = [];    % Covariance matrix [3×3]
            obj.ekfState.mu_history = [];   % History of estimates
            obj.ekfState.Sigma_history = {}; % History of covariances
            
            % Initialize plot handles
            obj.plotHandles = struct();
            obj.plotHandles.estimate = [];
            obj.plotHandles.ellipsoid = [];
            obj.plotHandles.detections = [];
            
            % EKF object will be created when running
            obj.ekf = [];
        end
        
        function setupCallbacks(obj)
            %SETUPCALLBACKS Attach callbacks to UI components
            
            % Button callbacks
            set(obj.handles.btnRunEKF, 'Callback', @(~,~) obj.runEKF());
            set(obj.handles.btnResetEKF, 'Callback', @(~,~) obj.resetEKF());
            
            % Visualization checkboxes
            set(obj.handles.chkShowEstimate, 'Callback', @(~,~) obj.updateVisualization());
            set(obj.handles.chkShowEllipsoid, 'Callback', @(~,~) obj.updateVisualization());
            set(obj.handles.chkShowDetections, 'Callback', @(~,~) obj.updateVisualization());
        end
        
        function runEKF(obj)
            %RUNEKF Execute EKF localization
            
            % Disable run button
            set(obj.handles.btnRunEKF, 'Enable', 'off');
            obj.setStatus('Initializing EKF...', [0, 0, 0.8]);
            
            % Clear sample plots from MODEL panel (keep landmarks, ground truth, occupancy map)
            children = get(obj.ax, 'Children');
            for i = 1:length(children)
                childType = class(children(i));
                % Delete Line objects with red dots (samples from MODEL panel)
                if strcmp(childType, 'matlab.graphics.chart.primitive.Line')
                    markerType = get(children(i), 'Marker');
                    lineColor = get(children(i), 'Color');
                    % Remove red sample dots (MarkerSize small, red color)
                    if strcmp(markerType, '.') && isequal(lineColor, [1, 0, 0])
                        delete(children(i));
                    end
                end
            end
            
            % Get MODEL panel data
            if ~isappdata(obj.fig, 'modelCallbacks')
                obj.setStatus('Error: Run MODEL simulation first', [0.8, 0, 0]);
                set(obj.handles.btnRunEKF, 'Enable', 'on');
                return;
            end
            
            modelCallbacks = getappdata(obj.fig, 'modelCallbacks');
            simState = modelCallbacks.simState;
            
            if isempty(simState.X_gt)
                obj.setStatus('Error: No trajectory data. Run MODEL first.', [0.8, 0, 0]);
                set(obj.handles.btnRunEKF, 'Enable', 'on');
                return;
            end
            
            % Get SENSORS panel data (landmarks)
            if ~isappdata(obj.fig, 'sensorsCallbacks')
                obj.setStatus('Error: SENSORS panel not initialized', [0.8, 0, 0]);
                set(obj.handles.btnRunEKF, 'Enable', 'on');
                return;
            end
            
            sensorsCallbacks = getappdata(obj.fig, 'sensorsCallbacks');
            landmarks = sensorsCallbacks.landmarks;
            
            if isempty(landmarks)
                obj.setStatus('Warning: No landmarks defined. Add landmarks in SENSORS panel.', [0.8, 0.5, 0]);
                set(obj.handles.btnRunEKF, 'Enable', 'on');
                return;
            end
            
            % Get EKF parameters
            P0_x = str2double(get(obj.handles.editP0x, 'String'));
            P0_y = str2double(get(obj.handles.editP0y, 'String'));
            P0_theta = str2double(get(obj.handles.editP0theta, 'String'));
            
            landmarkRadius = str2double(get(obj.handles.editLandmarkRadius, 'String'));
            
            % Get sensor parameters from SENSORS panel
            maxRange = sensorsCallbacks.sensorState.maxRange;
            fov = deg2rad(sensorsCallbacks.sensorState.fov);
            
            % Get measurement noise std from SENSORS panel (std, not variance)
            rangeNoiseStd = sensorsCallbacks.sensorState.rangeNoise;
            bearingNoiseStd = deg2rad(sensorsCallbacks.sensorState.angularNoise);  % Convert degrees to radians
            
            % Construct R matrix from variance (square of std)
            R_range = rangeNoiseStd^2;
            R_bearing = bearingNoiseStd^2;
            
            % Validate inputs
            if any(isnan([P0_x, P0_y, P0_theta, landmarkRadius]))
                obj.setStatus('Error: Invalid parameter values', [0.8, 0, 0]);
                set(obj.handles.btnRunEKF, 'Enable', 'on');
                return;
            end
            
            % Extract simulation data
            X_gt = simState.X_gt;
            U_list = simState.U_list;
            isDeadReckoning = simState.isDeadReckoning;
            dt = simState.dt;
            
            % Read current alpha values directly from MODEL panel sliders
            % This ensures we use the latest values even if user changed them
            % after running the simulation
            if isDeadReckoning
                alpha = zeros(6, 1);
                for i = 1:6
                    alpha(i) = get(modelCallbacks.handles.sliders.(sprintf('alpha%d', i)), 'Value');
                end
            else
                alpha = zeros(4, 1);
                for i = 1:4
                    alpha(i) = get(modelCallbacks.handles.sliders.(sprintf('alpha%d', i)), 'Value');
                end
            end
            
            % Determine number of steps
            if isDeadReckoning
                numSteps = size(U_list, 2);
            else
                U_odom = simState.U_odom;
                numSteps = size(U_odom, 2);
            end
            
            % Build params struct for EKF
            params = struct();
            if isDeadReckoning
                params.motionModel = 'deadreckoning';
            else
                params.motionModel = 'odometry';
            end
            params.dt = dt;
            params.alpha = alpha;
            params.R = diag([R_range, R_bearing]);
            params.sensorParams.maxRange = maxRange;
            params.sensorParams.fov = fov;
            
            % Initialize EKF object
            obj.ekf = EKFLocalizer(simState.x0, diag([P0_x, P0_y, P0_theta]), landmarks, params);
            
            % Initialize EKF state tracking
            obj.ekfState.mu = simState.x0;
            obj.ekfState.Sigma = diag([P0_x, P0_y, P0_theta]);
            obj.ekfState.currentStep = 0;
            obj.ekfState.isRunning = true;
            obj.ekfState.mu_history = zeros(3, numSteps + 1);
            obj.ekfState.mu_history(:, 1) = simState.x0;
            obj.ekfState.Sigma_history = cell(1, numSteps + 1);
            obj.ekfState.Sigma_history{1} = diag([P0_x, P0_y, P0_theta]);
            
            % Live update mode
            isLive = get(obj.handles.chkLiveEKF, 'Value');
            
            if isLive
                paceValue = str2double(get(obj.handles.editPace, 'String'));
                if isnan(paceValue) || paceValue < 0
                    paceValue = 0.1;
                end
            end
            
            obj.setStatus('Running EKF localization...', [0, 0.5, 0]);
            
            % Main EKF loop
            for step = 1:numSteps
                if ~obj.ekfState.isRunning
                    break;
                end
                
                obj.ekfState.currentStep = step;
                
                % Update MODEL panel's simState for cross-panel consistency
                if isappdata(obj.fig, 'modelCallbacks')
                    modelCallbacks.simState.currentStep = step;
                end
                
                % Get control input
                if isDeadReckoning
                    u = U_list(:, step);
                else
                    u = U_odom(:, step);
                end
                
                % Get current ground truth pose (for sensor simulation)
                currentGT = X_gt(:, step + 1);
                
                % Simulate lidar scan at ground truth pose
                halfFOV = fov / 2;
                resolution = deg2rad(sensorsCallbacks.sensorState.resolution);
                angles = -halfFOV:resolution:halfFOV;
                
                % Perform raycasting if occupancy map exists
                if ~isempty(simState.occMap)
                    poseRow = [currentGT(1), currentGT(2), currentGT(3)];
                    ranges = raycast(simState.occMap, poseRow, angles, maxRange);
                else
                    ranges = maxRange * ones(1, length(angles));
                end
                
                % Detect landmarks from scan
                [detectedLandmarks, measurements] = detectLandmarksFromScan(...
                    currentGT, ranges, angles, landmarks, landmarkRadius, rangeNoiseStd, bearingNoiseStd);
                
                % Convert measurements to EKF format
                if ~isempty(detectedLandmarks)
                    % z_actual = [measurements.ranges; measurements.bearings];  % [2K×1] interleaved
                    z_actual = [measurements.ranges,  measurements.bearings];
                    z_actual = reshape(z_actual.', [], 1);
                    z_actual = z_actual(:);  % Ensure column vector
                else
                    z_actual = [];
                end
                
                % EKF Prediction Step
                obj.ekf.predict(u);
                
                % EKF Update Step (if landmarks detected)
                if ~isempty(detectedLandmarks)
                    obj.ekf.update(z_actual, detectedLandmarks);
                end
                
                % Get current state from EKF
                [obj.ekfState.mu, obj.ekfState.Sigma] = obj.ekf.getState();
                
                % Store history
                obj.ekfState.mu_history(:, step + 1) = obj.ekfState.mu;
                obj.ekfState.Sigma_history{step + 1} = obj.ekfState.Sigma;
                
                % Visualization
                if isLive
                    obj.visualizeEKF(step + 1, X_gt, detectedLandmarks, landmarks);
                    
                    % Update LIDAR scan and visualize landmark measurements
                    if isappdata(obj.fig, 'sensorsCallbacks')
                        sensorsCallbacks = getappdata(obj.fig, 'sensorsCallbacks');
                        sensorsCallbacks.updateScanAtCurrentPose(currentGT, step);
                        
                        % Visualize actual landmark measurements used by EKF
                        if ~isempty(detectedLandmarks)
                            sensorsCallbacks.visualizeLandmarkMeasurements(currentGT, detectedLandmarks, measurements);
                        end
                    end
                    
                    pause(paceValue);
                    drawnow;
                end
            end
            
            % Final visualization if not live mode
            if ~isLive
                obj.visualizeEKF(numSteps + 1, X_gt, [], landmarks);
            end
            
            obj.ekfState.isRunning = false;
            obj.setStatus(sprintf('EKF Complete: %d steps', numSteps), [0, 0.5, 0]);
            set(obj.handles.btnRunEKF, 'Enable', 'on');
        end
        
        function visualizeEKF(obj, currentStep, X_gt, detectedLandmarks, allLandmarks)
            %VISUALIZEEKF Visualize EKF estimate and uncertainty
            
            % Clear previous EKF visualization
            obj.clearEKFVisualization();
            
            hold(obj.ax, 'on');
            
            % Get visualization options
            showEstimate = get(obj.handles.chkShowEstimate, 'Value');
            showEllipsoid = get(obj.handles.chkShowEllipsoid, 'Value');
            showDetections = get(obj.handles.chkShowDetections, 'Value');
            
            % Plot EKF estimate trajectory
            if showEstimate
                obj.plotHandles.estimate = plot(obj.ax, ...
                    obj.ekfState.mu_history(1, 1:currentStep), ...
                    obj.ekfState.mu_history(2, 1:currentStep), ...
                    'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
            end
            
            % Plot uncertainty ellipsoid at current pose
            if showEllipsoid && currentStep <= length(obj.ekfState.Sigma_history)
                mu_current = obj.ekfState.mu_history(:, currentStep);
                Sigma_current = obj.ekfState.Sigma_history{currentStep};
                
                % Validate Sigma_current before plotting
                if ~isempty(Sigma_current) && size(Sigma_current, 1) >= 2
                    confidenceLevel = str2double(get(obj.handles.editConfidence, 'String'));
                    if isnan(confidenceLevel) || confidenceLevel <= 0
                        confidenceLevel = 3;
                    end
                    
                    obj.plotHandles.ellipsoid = obj.plotUncertaintyEllipsoid(...
                        mu_current, Sigma_current, confidenceLevel);
                end
            end
            
            % Plot detected landmarks
            if showDetections && ~isempty(detectedLandmarks)
                obj.plotHandles.detections = plot(obj.ax, ...
                    detectedLandmarks(:, 2), detectedLandmarks(:, 3), ...
                    'mo', 'MarkerSize', 10, 'LineWidth', 2, ...
                    'DisplayName', 'Detected Landmarks');
            end
            
            % Update legend
            legend(obj.ax, 'Location', 'best');
        end
        
        function h = plotUncertaintyEllipsoid(obj, mu, Sigma, nSigma)
            %PLOTUNCERTAINTYELLIPSOID Plot 2D uncertainty ellipsoid
            %   mu: [3×1] state estimate
            %   Sigma: [3×3] covariance matrix
            %   nSigma: confidence level (1, 2, or 3 sigma)
            
            % Validate inputs
            if isempty(Sigma) || size(Sigma, 1) < 2 || size(Sigma, 2) < 2
                h = [];
                return;
            end
            
            % Extract position covariance (2×2)
            Sigma_xy = Sigma(1:2, 1:2);
            
            % Eigenvalue decomposition
            [V, D] = eig(Sigma_xy);
            
            % Semi-axes lengths
            a = nSigma * sqrt(D(1, 1));  % Semi-major or semi-minor axis
            b = nSigma * sqrt(D(2, 2));  % Semi-major or semi-minor axis
            
            % Rotation angle
            angle = atan2(V(2, 1), V(1, 1));
            
            % Generate ellipse points
            theta = linspace(0, 2*pi, 50);
            ellipse_x = a * cos(theta);
            ellipse_y = b * sin(theta);
            
            % Rotation matrix
            R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
            ellipse_pts = R * [ellipse_x; ellipse_y];
            
            % Translate to mean position
            ellipse_x = ellipse_pts(1, :) + mu(1);
            ellipse_y = ellipse_pts(2, :) + mu(2);
            
            % Plot ellipse
            h = plot(obj.ax, ellipse_x, ellipse_y, ...
                'b--', 'LineWidth', 1.5, 'DisplayName', ...
                sprintf('%dσ Uncertainty', nSigma));
        end
        
        function clearEKFVisualization(obj)
            %CLEAREKFVISUALIZATION Clear EKF visualization objects
            
            if ~isempty(obj.plotHandles.estimate) && isgraphics(obj.plotHandles.estimate)
                delete(obj.plotHandles.estimate);
                obj.plotHandles.estimate = [];
            end
            
            if ~isempty(obj.plotHandles.ellipsoid) && isgraphics(obj.plotHandles.ellipsoid)
                delete(obj.plotHandles.ellipsoid);
                obj.plotHandles.ellipsoid = [];
            end
            
            if ~isempty(obj.plotHandles.detections) && isgraphics(obj.plotHandles.detections)
                delete(obj.plotHandles.detections);
                obj.plotHandles.detections = [];
            end
        end
        
        function resetEKF(obj)
            %RESETEKF Reset EKF state and visualization
            
            obj.ekfState.isRunning = false;
            obj.ekfState.currentStep = 0;
            obj.ekfState.mu = [];
            obj.ekfState.Sigma = [];
            obj.ekfState.mu_history = [];
            obj.ekfState.Sigma_history = {};
            
            % Clear EKF object
            obj.ekf = [];
            
            obj.clearEKFVisualization();
            
            obj.setStatus('EKF Reset', [0.5, 0.5, 0.5]);
        end
        
        function updateVisualization(obj)
            %UPDATEVISUALIZATION Update visualization based on checkbox states
            
            if ~isempty(obj.ekfState.mu_history) && ~isempty(obj.ekfState.Sigma_history)
                currentStep = size(obj.ekfState.mu_history, 2);
                
                % Ensure Sigma_history has valid data at current step
                if currentStep < 1 || currentStep > length(obj.ekfState.Sigma_history)
                    return;
                end
                
                % Get ground truth from MODEL panel
                if isappdata(obj.fig, 'modelCallbacks')
                    modelCallbacks = getappdata(obj.fig, 'modelCallbacks');
                    X_gt = modelCallbacks.simState.X_gt;
                else
                    X_gt = [];
                end
                
                % Get landmarks from SENSORS panel
                if isappdata(obj.fig, 'sensorsCallbacks')
                    sensorsCallbacks = getappdata(obj.fig, 'sensorsCallbacks');
                    landmarks = sensorsCallbacks.landmarks;
                else
                    landmarks = [];
                end
                
                obj.visualizeEKF(currentStep, X_gt, [], landmarks);
            end
        end
        
        function setStatus(obj, message, color)
            %SETSTATUS Update status display
            set(obj.handles.txtStatus, 'String', ['Status: ', message]);
            set(obj.handles.txtStatus, 'ForegroundColor', color);
        end
    end
end

