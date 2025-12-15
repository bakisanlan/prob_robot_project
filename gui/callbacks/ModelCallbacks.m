classdef ModelCallbacks < handle
%MODELCALLBACKS Handles all callbacks for the MODEL panel
%
%   This class encapsulates all callback functions related to the MODEL panel,
%   including simulation control, model selection, and result saving.
%
%   Usage:
%       callbacks = ModelCallbacks(ax, handles, simState);
%       callbacks.setupCallbacks();

    properties
        ax              % Main axes handle
        handles         % Struct containing all MODEL panel UI handles
        simState        % Simulation state structure (handle class or struct reference)
        fig             % Parent figure handle
        panelButtons    % Panel selection buttons [MODEL, SENSORS, GNC]
        allPanelComponents  % Cell array of {modelComponents, sensorsComponents, gncComponents}
    end
    
    methods
        function obj = ModelCallbacks(ax, handles, fig)
            %MODELCALLBACKS Constructor
            obj.ax = ax;
            obj.handles = handles;
            obj.fig = fig;
            
            % Initialize simulation state
            obj.simState = struct();
            obj.simState.isRunning = false;
            obj.simState.isPaused = false;
            obj.simState.isFinished = false;
            obj.simState.currentStep = 0;
            obj.simState.X_samples = [];
            obj.simState.X_gt = [];
            obj.simState.U_list = [];
            obj.simState.U_odom = [];
            obj.simState.x0 = [];
            obj.simState.dt = 0;
            obj.simState.numSamples = 0;
            obj.simState.alpha = [];
            obj.simState.isDeadReckoning = true;
            obj.simState.trajectoryType = 1;
            obj.simState.occMap = [];
        end
        
        function setupCallbacks(obj)
            %SETUPCALLBACKS Attach callbacks to UI components
            
            % Model selection callbacks
            set(obj.handles.btnDeadReckoning, 'Callback', @(src,~) obj.modelSelectionCallback(src));
            set(obj.handles.btnOdometry, 'Callback', @(src,~) obj.modelSelectionCallback(src));
            
            % Slider callbacks
            for i = 1:6
                sliderHandle = obj.handles.sliders.(sprintf('alpha%d', i));
                set(sliderHandle, 'Callback', @(src,~) obj.sliderCallback(src, i));
            end
            
            % Live simulation checkbox
            set(obj.handles.chkLiveSimulation, 'Callback', @(~,~) obj.liveSimulationCallback());
            
            % Simulation control buttons
            set(obj.handles.btnRun, 'Callback', @(~,~) obj.runSimulation());
            set(obj.handles.btnStop, 'Callback', @(~,~) obj.stopSimulation());
            set(obj.handles.btnContinue, 'Callback', @(~,~) obj.continueSimulation());
            set(obj.handles.btnReset, 'Callback', @(~,~) obj.resetSimulation());
            set(obj.handles.btnSaveResults, 'Callback', @(~,~) obj.saveResults());
            
            % Initialize slider visibility
            obj.updateSliderVisibility();
        end
        
        function modelSelectionCallback(obj, src)
            %MODELSELECTIONCALLBACK Toggle between Dead Reckoning and Odometry
            if src == obj.handles.btnDeadReckoning
                set(obj.handles.btnDeadReckoning, 'Value', 1);
                set(obj.handles.btnOdometry, 'Value', 0);
            else
                set(obj.handles.btnDeadReckoning, 'Value', 0);
                set(obj.handles.btnOdometry, 'Value', 1);
            end
            obj.updateSliderVisibility();
        end
        
        function updateSliderVisibility(obj)
            %UPDATESLIDERVISIBILITY Show/hide sliders based on model type
            isDeadReckoning = get(obj.handles.btnDeadReckoning, 'Value');
            
            if isDeadReckoning
                numSliders = 6;
            else
                numSliders = 4;
            end
            
            for i = 1:6
                if i <= numSliders
                    set(obj.handles.sliderLabels.(sprintf('alpha%d', i)), 'Visible', 'on');
                    set(obj.handles.sliders.(sprintf('alpha%d', i)), 'Visible', 'on');
                    set(obj.handles.sliderValues.(sprintf('alpha%d', i)), 'Visible', 'on');
                else
                    set(obj.handles.sliderLabels.(sprintf('alpha%d', i)), 'Visible', 'off');
                    set(obj.handles.sliders.(sprintf('alpha%d', i)), 'Visible', 'off');
                    set(obj.handles.sliderValues.(sprintf('alpha%d', i)), 'Visible', 'off');
                end
            end
        end
        
        function sliderCallback(obj, src, idx)
            %SLIDERCALLBACK Update slider value display
            val = get(src, 'Value');
            set(obj.handles.sliderValues.(sprintf('alpha%d', idx)), 'String', sprintf('%.4f', val));
        end
        
        function liveSimulationCallback(obj)
            %LIVESIMULATIONCALLBACK Toggle live simulation controls
            isLive = get(obj.handles.chkLiveSimulation, 'Value');
            if isLive
                set(obj.handles.btnStop, 'Visible', 'on');
                set(obj.handles.btnContinue, 'Visible', 'on');
                set(obj.handles.btnReset, 'Visible', 'on');
                set(obj.handles.txtPaceLabel, 'Visible', 'on');
                set(obj.handles.editPace, 'Visible', 'on');
            else
                set(obj.handles.btnStop, 'Visible', 'off');
                set(obj.handles.btnContinue, 'Visible', 'off');
                set(obj.handles.btnReset, 'Visible', 'off');
                set(obj.handles.txtPaceLabel, 'Visible', 'off');
                set(obj.handles.editPace, 'Visible', 'off');
            end
        end
        
        function runSimulation(obj)
            %RUNSIMULATION Main simulation execution
            
            % Disable run button
            set(obj.handles.btnRun, 'Enable', 'off');
            
            % Clear previous plot
            cla(obj.ax);
            hold(obj.ax, 'on');
            
            % Get parameters
            numSamples = str2double(get(obj.handles.editSamples, 'String'));
            if isnan(numSamples) || numSamples < 1
                errordlg('Please enter a valid number of samples (positive integer)');
                set(obj.handles.btnRun, 'Enable', 'on');
                return;
            end
            numSamples = round(numSamples);
            
            isDeadReckoning = get(obj.handles.btnDeadReckoning, 'Value');
            trajectoryType = get(obj.handles.popupTrajectory, 'Value');
            isLive = get(obj.handles.chkLiveSimulation, 'Value');
            
            % Generate trajectory
            switch trajectoryType
                case 1
                    [X_gt, U_list, x0, dt] = generateCircularTrajectory();
                    obj.simState.occMap = [];
                case 2
                    [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
                    obj.simState.occMap = [];
                case 3
                    [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory();
                    obj.simState.occMap = occMap;
                    show(occMap, 'Parent', obj.ax);
                    hold(obj.ax, 'on');
                otherwise
                    errordlg('Unknown trajectory type');
                    set(obj.handles.btnRun, 'Enable', 'on');
                    return;
            end
            
            % Update simulation state
            obj.simState.isRunning = true;
            obj.simState.isPaused = false;
            obj.simState.isFinished = false;
            obj.simState.currentStep = 0;
            obj.simState.X_gt = X_gt;
            obj.simState.U_list = U_list;
            obj.simState.x0 = x0;
            obj.simState.dt = dt;
            obj.simState.numSamples = numSamples;
            obj.simState.isDeadReckoning = isDeadReckoning;
            obj.simState.trajectoryType = trajectoryType;
            
            if isDeadReckoning
                alpha = zeros(6, 1);
                for i = 1:6
                    alpha(i) = get(obj.handles.sliders.(sprintf('alpha%d', i)), 'Value');
                end
                obj.simState.alpha = alpha;
                
                size_U = size(U_list, 2);
                obj.simState.X_samples = repmat(x0, 1, numSamples, size_U+1);
                
                modelName = 'Dead Reckoning';
            else
                alpha = zeros(4, 1);
                for i = 1:4
                    alpha(i) = get(obj.handles.sliders.(sprintf('alpha%d', i)), 'Value');
                end
                obj.simState.alpha = alpha;
                
                % Convert to odometry control format
                U_odom = [];
                x_odom = x0;
                
                for i = 1:length(U_list(1,:))
                    u_odom = [X_gt(:, i); X_gt(:, i+1)];
                    x_odom = GTodometryMotionModel(x_odom, u_odom);
                    U_odom = [U_odom, u_odom];
                end
                
                obj.simState.U_odom = U_odom;
                size_U = size(U_odom, 2);
                obj.simState.X_samples = repmat(x0, 1, numSamples, size_U+1);
                
                modelName = 'Odometry';
            end
            
            if isLive
                % Live simulation mode
                set(obj.handles.btnStop, 'Enable', 'on');
                set(obj.handles.btnContinue, 'Enable', 'off');
                
                plot(obj.ax, X_gt(1,1), X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
                
                trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
                title(obj.ax, sprintf('%s Model - %s Trajectory - Live Simulation', ...
                                 modelName, trajectoryNames{trajectoryType}));
                legend(obj.ax, 'Location', 'best');
                grid(obj.ax, 'on');
                axis(obj.ax, 'equal');
                
                obj.runLiveSimulation();
            else
                % Batch mode
                if isDeadReckoning
                    size_U = size(U_list, 2);
                    for i = 1:size_U
                        for sample = 1:numSamples
                            obj.simState.X_samples(:, sample, i+1) = sampleDeadReckoningMotionModel(...
                                obj.simState.X_samples(:, sample, i), U_list(:, i), alpha, dt, obj.simState.occMap);
                        end
                    end
                else
                    size_U = size(obj.simState.U_odom, 2);
                    for i = 1:size_U
                        for sample = 1:numSamples
                            obj.simState.X_samples(:, sample, i+1) = sampleOdometryMotionModel(...
                                obj.simState.X_samples(:, sample, i), obj.simState.U_odom(:, i), alpha, obj.simState.occMap);
                        end
                    end
                end
                
                % Plot all results
                xpos = obj.simState.X_samples(1, :, :);
                xpos = xpos(:, :);
                ypos = obj.simState.X_samples(2, :, :);
                ypos = ypos(:, :);
                
                plot(obj.ax, xpos, ypos, 'r.', 'MarkerSize', 2, 'DisplayName', 'Samples');
                plot(obj.ax, X_gt(1,:), X_gt(2,:), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
                
                trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
                title(obj.ax, sprintf('%s Model - %s Trajectory - %d Samples', ...
                                 modelName, trajectoryNames{trajectoryType}, numSamples));
                legend(obj.ax, 'Location', 'best');
                grid(obj.ax, 'on');
                axis(obj.ax, 'equal');
                
                set(obj.handles.btnRun, 'Enable', 'on');
                set(obj.handles.btnSaveResults, 'Enable', 'on');
            end
        end
        
        function runLiveSimulation(obj)
            %RUNLIVESIMULATION Execute live step-by-step simulation
            
            if ~obj.simState.isFinished
                % Get pace
                paceValue = str2double(get(obj.handles.editPace, 'String'));
                if isnan(paceValue) || paceValue < 0
                    paceValue = 0.1;
                    set(obj.handles.editPace, 'String', '0.1');
                end
                
                if obj.simState.isDeadReckoning
                    size_U = size(obj.simState.U_list, 2);
                else
                    size_U = size(obj.simState.U_odom, 2);
                end
                
                for step = (obj.simState.currentStep + 1):size_U
                    if ~obj.simState.isRunning
                        if ~obj.simState.isPaused
                            break;
                        else
                            while obj.simState.isPaused
                                return;
                            end
                        end
                    end
                    
                    obj.simState.currentStep = step;
                    
                    % Propagate samples
                    if obj.simState.isDeadReckoning
                        for sample = 1:obj.simState.numSamples
                            x_new = sampleDeadReckoningMotionModel(...
                                obj.simState.X_samples(:, sample, step), ...
                                obj.simState.U_list(:, step), ...
                                obj.simState.alpha, obj.simState.dt, obj.simState.occMap);
                            obj.simState.X_samples(:, sample, step+1) = x_new;
                        end
                    else
                        for sample = 1:obj.simState.numSamples
                            x_new = sampleOdometryMotionModel(...
                                obj.simState.X_samples(:, sample, step), ...
                                obj.simState.U_odom(:, step), ...
                                obj.simState.alpha, obj.simState.occMap);
                            obj.simState.X_samples(:, sample, step+1) = x_new;
                        end
                    end
                    
                    % Plot current samples
                    xpos = obj.simState.X_samples(1, :, 1:(step+1));
                    xpos = xpos(:, :);
                    ypos = obj.simState.X_samples(2, :, 1:(step+1));
                    ypos = ypos(:, :);
                    
                    % Clear line objects only
                    children = get(obj.ax, 'Children');
                    for i = 1:length(children)
                        childType = class(children(i));
                        if strcmp(childType, 'matlab.graphics.chart.primitive.Line')
                            markerType = get(children(i), 'Marker');
                            if strcmp(markerType, '.') || strcmp(markerType, 'o')
                                delete(children(i));
                            end
                        end
                    end
                    
                    plot(obj.ax, xpos, ypos, 'r.', 'MarkerSize', 2, 'DisplayName', 'Samples');
                    plot(obj.ax, obj.simState.X_gt(1, 1:(step+1)), obj.simState.X_gt(2, 1:(step+1)), ...
                         'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
                    
                    trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
                    modelName = 'Odometry';
                    if obj.simState.isDeadReckoning
                        modelName = 'Dead Reckoning';
                    end
                    title(obj.ax, sprintf('%s Model - %s Trajectory - Step %d/%d', ...
                                     modelName, trajectoryNames{obj.simState.trajectoryType}, step, size_U));
                    
                    % Update LIDAR scan if SENSORS panel has auto-update enabled
                    if isappdata(obj.fig, 'sensorsCallbacks')
                        sensorsCallbacks = getappdata(obj.fig, 'sensorsCallbacks');
                        sensorsCallbacks.updateScanAtCurrentPose();
                    end
                    
                    drawnow;
                    pause(paceValue);
                end
            end
            
            % Simulation finished
            obj.simState.isRunning = false;
            obj.simState.isFinished = true;
            set(obj.handles.btnRun, 'Enable', 'on');
            set(obj.handles.btnStop, 'Enable', 'off');
            set(obj.handles.btnContinue, 'Enable', 'off');
            set(obj.handles.btnSaveResults, 'Enable', 'on');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            modelName = 'Odometry';
            if obj.simState.isDeadReckoning
                modelName = 'Dead Reckoning';
            end
            title(obj.ax, sprintf('%s Model - %s Trajectory - Complete (%d samples)', ...
                             modelName, trajectoryNames{obj.simState.trajectoryType}, obj.simState.numSamples));
        end
        
        function stopSimulation(obj)
            %STOPSIMULATION Pause live simulation
            obj.simState.isRunning = false;
            obj.simState.isPaused = true;
            set(obj.handles.btnStop, 'Enable', 'off');
            set(obj.handles.btnContinue, 'Enable', 'on');
            set(obj.handles.btnReset, 'Enable', 'on');
            set(obj.handles.btnRun, 'Enable', 'off');
        end
        
        function continueSimulation(obj)
            %CONTINUESIMULATION Resume paused simulation
            obj.simState.isRunning = true;
            obj.simState.isPaused = false;
            set(obj.handles.btnStop, 'Enable', 'on');
            set(obj.handles.btnContinue, 'Enable', 'off');
            set(obj.handles.btnReset, 'Enable', 'off');
            set(obj.handles.btnRun, 'Enable', 'off');
            
            obj.runLiveSimulation();
        end
        
        function resetSimulation(obj)
            %RESETSIMULATION Reset to initial state
            obj.simState.isRunning = false;
            obj.simState.isPaused = false;
            obj.simState.isFinished = false;
            obj.simState.currentStep = 0;
            
            % Reset samples
            if obj.simState.isDeadReckoning
                size_U = size(obj.simState.U_list, 2);
                obj.simState.X_samples = repmat(obj.simState.x0, 1, obj.simState.numSamples, size_U+1);
            else
                size_U = size(obj.simState.U_odom, 2);
                obj.simState.X_samples = repmat(obj.simState.x0, 1, obj.simState.numSamples, size_U+1);
            end
            
            % Clear and reset plot
            cla(obj.ax);
            hold(obj.ax, 'on');
            
            if ~isempty(obj.simState.occMap)
                show(obj.simState.occMap, 'Parent', obj.ax);
                hold(obj.ax, 'on');
            end
            
            plot(obj.ax, obj.simState.X_gt(1,1), obj.simState.X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            modelName = 'Odometry';
            if obj.simState.isDeadReckoning
                modelName = 'Dead Reckoning';
            end
            title(obj.ax, sprintf('%s Model - %s Trajectory - Reset', ...
                             modelName, trajectoryNames{obj.simState.trajectoryType}));
            legend(obj.ax, 'Location', 'best');
            grid(obj.ax, 'on');
            axis(obj.ax, 'equal');
            
            set(obj.handles.btnRun, 'Enable', 'on');
            set(obj.handles.btnStop, 'Enable', 'off');
            set(obj.handles.btnContinue, 'Enable', 'off');
            set(obj.handles.btnReset, 'Enable', 'off');
        end
        
        function saveResults(obj)
            %SAVERESULTS Save simulation results to files
            
            if isempty(obj.simState.X_samples)
                errordlg('No simulation data to save. Please run a simulation first.');
                return;
            end
            
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            trajectoryNames = {'Circular', 'Rectangle', 'RectangleObstacles'};
            trajectoryName = trajectoryNames{obj.simState.trajectoryType};
            
            if obj.simState.isDeadReckoning
                modelName = 'DeadReckoning';
            else
                modelName = 'Odometry';
            end
            
            baseFilename = sprintf('RobotSim_%s_%s_%s', modelName, trajectoryName, timestamp);
            
            [filename, pathname] = uiputfile(...
                {'*.mat', 'MATLAB Data (*.mat)'; '*.txt', 'Text File (*.txt)'}, ...
                'Save Simulation Results', ...
                fullfile(pwd, baseFilename));
            
            if filename == 0
                return;
            end
            
            [~, filenameNoExt, ~] = fileparts(filename);
            
            try
                % Save figure as PNG
                figureFilename = fullfile(pathname, [filenameNoExt, '.png']);
                exportgraphics(obj.fig, figureFilename, 'Resolution', 300);
                
                % Save data as MAT
                matFilename = fullfile(pathname, [filenameNoExt, '.mat']);
                simulationData = struct();
                simulationData.X_samples = obj.simState.X_samples;
                simulationData.X_gt = obj.simState.X_gt;
                simulationData.U_list = obj.simState.U_list;
                simulationData.x0 = obj.simState.x0;
                simulationData.dt = obj.simState.dt;
                simulationData.alpha = obj.simState.alpha;
                simulationData.numSamples = obj.simState.numSamples;
                simulationData.modelType = modelName;
                simulationData.trajectoryType = trajectoryName;
                simulationData.timestamp = timestamp;
                
                if ~obj.simState.isDeadReckoning
                    simulationData.U_odom = obj.simState.U_odom;
                end
                
                save(matFilename, 'simulationData');
                
                % Save config as TXT
                txtFilename = fullfile(pathname, [filenameNoExt, '_config.txt']);
                fid = fopen(txtFilename, 'w');
                
                fprintf(fid, '========================================\n');
                fprintf(fid, 'Robot Motion Model Simulation Results\n');
                fprintf(fid, '========================================\n\n');
                fprintf(fid, 'Timestamp: %s\n\n', datestr(now));
                fprintf(fid, 'SIMULATION CONFIGURATION\n');
                fprintf(fid, '------------------------\n');
                fprintf(fid, 'Motion Model: %s\n', modelName);
                fprintf(fid, 'Trajectory Type: %s\n', strrep(trajectoryName, '_', ' '));
                fprintf(fid, 'Number of Samples: %d\n', obj.simState.numSamples);
                fprintf(fid, 'Time Step (dt): %.4f seconds\n', obj.simState.dt);
                fprintf(fid, 'Number of Control Steps: %d\n', size(obj.simState.U_list, 2));
                fprintf(fid, '\n');
                
                fprintf(fid, 'NOISE PARAMETERS (Alpha)\n');
                fprintf(fid, '-------------------------\n');
                if obj.simState.isDeadReckoning
                    fprintf(fid, 'α1 (v→v noise): %.6f\n', obj.simState.alpha(1));
                    fprintf(fid, 'α2 (ω→v noise): %.6f\n', obj.simState.alpha(2));
                    fprintf(fid, 'α3 (v→ω noise): %.6f\n', obj.simState.alpha(3));
                    fprintf(fid, 'α4 (ω→ω noise): %.6f\n', obj.simState.alpha(4));
                    fprintf(fid, 'α5 (v→γ noise): %.6f\n', obj.simState.alpha(5));
                    fprintf(fid, 'α6 (ω→γ noise): %.6f\n', obj.simState.alpha(6));
                else
                    fprintf(fid, 'α1 (rot→rot noise): %.6f\n', obj.simState.alpha(1));
                    fprintf(fid, 'α2 (trans→rot noise): %.6f\n', obj.simState.alpha(2));
                    fprintf(fid, 'α3 (trans→trans noise): %.6f\n', obj.simState.alpha(3));
                    fprintf(fid, 'α4 (rot→trans noise): %.6f\n', obj.simState.alpha(4));
                end
                fprintf(fid, '\n');
                
                fprintf(fid, 'INITIAL POSE\n');
                fprintf(fid, '------------\n');
                fprintf(fid, 'x: %.4f m\n', obj.simState.x0(1));
                fprintf(fid, 'y: %.4f m\n', obj.simState.x0(2));
                fprintf(fid, 'θ: %.4f rad (%.2f deg)\n', obj.simState.x0(3), rad2deg(obj.simState.x0(3)));
                fprintf(fid, '\n');
                
                fprintf(fid, 'FINAL GROUND TRUTH POSE\n');
                fprintf(fid, '-----------------------\n');
                finalGT = obj.simState.X_gt(:, end);
                fprintf(fid, 'x: %.4f m\n', finalGT(1));
                fprintf(fid, 'y: %.4f m\n', finalGT(2));
                fprintf(fid, 'θ: %.4f rad (%.2f deg)\n', finalGT(3), rad2deg(finalGT(3)));
                fprintf(fid, '\n');
                
                if obj.simState.trajectoryType == 3
                    fprintf(fid, 'OBSTACLE INFORMATION\n');
                    fprintf(fid, '--------------------\n');
                    fprintf(fid, 'Obstacle Map: Yes\n');
                    fprintf(fid, 'Map Size: 20m x 10m\n');
                    fprintf(fid, 'Resolution: 0.5m\n');
                    fprintf(fid, 'Obstacle 1: 4x4m square at (2, 4)\n');
                    fprintf(fid, 'Obstacle 2: 4x4m square at (14, 4)\n');
                    fprintf(fid, '\n');
                end
                
                fprintf(fid, 'OUTPUT FILES\n');
                fprintf(fid, '------------\n');
                fprintf(fid, 'Figure: %s\n', [filenameNoExt, '.png']);
                fprintf(fid, 'Data: %s\n', [filenameNoExt, '.mat']);
                fprintf(fid, 'Config: %s\n', [filenameNoExt, '_config.txt']);
                fprintf(fid, '\n');
                fprintf(fid, '========================================\n');
                fprintf(fid, 'End of Report\n');
                fprintf(fid, '========================================\n');
                
                fclose(fid);
                
                msgbox(sprintf('Results saved successfully!\n\nFiles created:\n• %s.png\n• %s.mat\n• %s_config.txt', ...
                       filenameNoExt, filenameNoExt, filenameNoExt), ...
                       'Save Successful', 'help');
                
            catch ME
                errordlg(sprintf('Error saving results: %s', ME.message), 'Save Error');
            end
        end
    end
end
