function panel = createModelPanel(fig, ax, dims, sharedState)
    %CREATEMODELPANEL Create the MODEL panel components
    
    panel = struct();
    panel.fig = fig;
    panel.ax = ax;
    % Store handle to shared state, not a copy
    panel.sharedStateHandle = sharedState;
    
    % Initialize simulation state flags as persistent variables accessible to nested functions
    simState = struct();
    simState.isRunning = false;
    simState.isPaused = false;
    simState.isFinished = false;
    simState.currentStep = 0;
    
    % Model selection
    txtModelLabel = uicontrol('Parent', fig, 'Style', 'text', ...
              'String', 'Motion Model:', ...
              'Position', [dims.panelX, dims.figHeight*0.857, dims.panelWidth*0.6, dims.figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    btnDeadReckoning = uicontrol('Parent', fig, 'Style', 'radiobutton', ...
                                  'String', 'Dead Reckoning', ...
                                  'Position', [dims.panelX, dims.figHeight*0.814, dims.panelWidth*0.5, dims.figHeight*0.036], ...
                                  'Value', 1);
    
    btnOdometry = uicontrol('Parent', fig, 'Style', 'radiobutton', ...
                            'String', 'Odometry', ...
                            'Position', [dims.panelX, dims.figHeight*0.771, dims.panelWidth*0.5, dims.figHeight*0.036], ...
                            'Value', 0);
    
    % Trajectory selection
    txtTrajectoryLabel = uicontrol('Parent', fig, 'Style', 'text', ...
              'String', 'Trajectory Type:', ...
              'Position', [dims.panelX, dims.figHeight*0.72, dims.panelWidth*0.6, dims.figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    popupTrajectory = uicontrol('Parent', fig, 'Style', 'popupmenu', ...
                                'String', {'Circular', 'Rectangle', 'Rectangle with Obstacles'}, ...
                                'Position', [dims.panelX, dims.figHeight*0.684, dims.panelWidth*0.7, dims.figHeight*0.036], ...
                                'Value', 1);
    
    % Number of samples
    txtSamplesLabel = uicontrol('Parent', fig, 'Style', 'text', ...
              'String', 'Number of Samples:', ...
              'Position', [dims.panelX, dims.figHeight*0.63, dims.panelWidth*0.6, dims.figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    editSamples = uicontrol('Parent', fig, 'Style', 'edit', ...
                            'String', '1000', ...
                            'Position', [dims.panelX, dims.figHeight*0.594, dims.panelWidth*0.4, dims.figHeight*0.036], ...
                            'HorizontalAlignment', 'center');
    
    % Alpha parameters
    txtAlphaLabel = uicontrol('Parent', fig, 'Style', 'text', ...
                              'String', 'Alpha Parameters:', ...
                              'Position', [dims.panelX, dims.figHeight*0.53, dims.panelWidth*0.6, dims.figHeight*0.036], ...
                              'HorizontalAlignment', 'left', ...
                              'FontSize', 10, 'FontWeight', 'bold');
    
    % Create sliders
    sliders = struct();
    sliderLabels = struct();
    sliderValues = struct();
    sliderSpacing = dims.figHeight * 0.046;
    sliderHeight = dims.figHeight * 0.029;
    
    for i = 1:6
        yPos = dims.figHeight*0.5 - (i-1)*sliderSpacing;
        
        sliderLabels.(sprintf('alpha%d', i)) = uicontrol('Parent', fig, 'Style', 'text', ...
            'String', sprintf('α%d:', i), ...
            'Position', [dims.panelX, yPos, dims.panelWidth*0.12, sliderHeight], ...
            'HorizontalAlignment', 'left', ...
            'Tag', sprintf('sliderLabel%d', i));
        
        sliders.(sprintf('alpha%d', i)) = uicontrol('Parent', fig, 'Style', 'slider', ...
            'Min', 0, 'Max', 0.01, 'Value', 0.001, ...
            'Position', [dims.panelX+dims.panelWidth*0.14, yPos, dims.panelWidth*0.64, sliderHeight], ...
            'Tag', sprintf('slider%d', i));
        
        sliderValues.(sprintf('alpha%d', i)) = uicontrol('Parent', fig, 'Style', 'text', ...
            'String', '0.001', ...
            'Position', [dims.panelX+dims.panelWidth*0.79, yPos, dims.panelWidth*0.18, sliderHeight], ...
            'HorizontalAlignment', 'left', ...
            'Tag', sprintf('sliderValue%d', i));
    end
    
    % Live simulation controls
    chkLiveSimulation = uicontrol('Parent', fig, 'Style', 'checkbox', ...
                                  'String', 'Live Simulation', ...
                                  'Position', [dims.panelX, dims.figHeight*0.15, dims.panelWidth*0.5, dims.figHeight*0.036], ...
                                  'Value', 0, ...
                                  'FontSize', 10);
    
    txtPaceLabel = uicontrol('Parent', fig, 'Style', 'text', ...
                             'String', 'Pace (s):', ...
                             'Position', [dims.panelX, dims.figHeight*0.11, dims.panelWidth*0.3, dims.figHeight*0.03], ...
                             'HorizontalAlignment', 'left', ...
                             'FontSize', 9, ...
                             'Visible', 'off');
    
    editPace = uicontrol('Parent', fig, 'Style', 'edit', ...
                         'String', '0.1', ...
                         'Position', [dims.panelX+dims.panelWidth*0.32, dims.figHeight*0.11, dims.panelWidth*0.25, dims.figHeight*0.03], ...
                         'HorizontalAlignment', 'center', ...
                         'FontSize', 9, ...
                         'Visible', 'off');
    
    % Simulation buttons
    btnRun = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                       'String', 'Run', ...
                       'Position', [dims.panelX, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.4, 0.7, 0.4]);
    
    btnStop = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                       'String', 'Stop', ...
                       'Position', [dims.panelX+dims.panelWidth*0.25, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.9, 0.4, 0.4], ...
                       'Enable', 'off', ...
                       'Visible', 'off');
    
    btnContinue = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                       'String', 'Continue', ...
                       'Position', [dims.panelX+dims.panelWidth*0.5, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.4, 0.6, 0.9], ...
                       'Enable', 'off', ...
                       'Visible', 'off');
    
    btnReset = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                       'String', 'Reset', ...
                       'Position', [dims.panelX+dims.panelWidth*0.75, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.9, 0.7, 0.2], ...
                       'Enable', 'off', ...
                       'Visible', 'off');
    
    btnSave = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                       'String', 'Save Results', ...
                       'Position', [dims.panelX, dims.figHeight*0.01, dims.panelWidth*0.5, dims.figHeight*0.04], ...
                       'FontSize', 10, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.3, 0.8, 0.8], ...
                       'Enable', 'off');
    
    % Store all components
    sliderLabelsList = struct2cell(sliderLabels);
    slidersList = struct2cell(sliders);
    sliderValuesList = struct2cell(sliderValues);
    
    panel.components = [txtModelLabel, btnDeadReckoning, btnOdometry, ...
                        txtTrajectoryLabel, popupTrajectory, ...
                        txtSamplesLabel, editSamples, txtAlphaLabel, ...
                        chkLiveSimulation, txtPaceLabel, editPace, ...
                        btnRun, btnStop, btnContinue, btnReset, btnSave, ...
                        sliderLabelsList{:}, slidersList{:}, sliderValuesList{:}];
    
    % Store individual references
    panel.txtModelLabel = txtModelLabel;
    panel.btnDeadReckoning = btnDeadReckoning;
    panel.btnOdometry = btnOdometry;
    panel.txtTrajectoryLabel = txtTrajectoryLabel;
    panel.popupTrajectory = popupTrajectory;
    panel.txtSamplesLabel = txtSamplesLabel;
    panel.editSamples = editSamples;
    panel.txtAlphaLabel = txtAlphaLabel;
    panel.sliders = sliders;
    panel.sliderLabels = sliderLabels;
    panel.sliderValues = sliderValues;
    panel.chkLiveSimulation = chkLiveSimulation;
    panel.txtPaceLabel = txtPaceLabel;
    panel.editPace = editPace;
    panel.btnRun = btnRun;
    panel.btnStop = btnStop;
    panel.btnContinue = btnContinue;
    panel.btnReset = btnReset;
    panel.btnSave = btnSave;
    
    % Set callbacks
    set(btnDeadReckoning, 'Callback', @modelSelectionCallback);
    set(btnOdometry, 'Callback', @modelSelectionCallback);
    set(chkLiveSimulation, 'Callback', @liveSimulationCallback);
    
    for i = 1:6
        set(sliders.(sprintf('alpha%d', i)), 'Callback', @(src,evt)sliderCallback(src, i));
    end
    
    set(btnRun, 'Callback', @runSimulation);
    set(btnStop, 'Callback', @stopSimulation);
    set(btnContinue, 'Callback', @continueSimulation);
    set(btnReset, 'Callback', @resetSimulation);
    set(btnSave, 'Callback', @saveResults);
    
    % Initialize slider visibility - MOVED TO END after all variables are defined
    updateSliderVisibility();

    %% Nested Callback Functions (defined inside createModelPanel)
    
    function modelSelectionCallback(src, ~)
        if src == btnDeadReckoning
            set(btnDeadReckoning, 'Value', 1);
            set(btnOdometry, 'Value', 0);
        else
            set(btnDeadReckoning, 'Value', 0);
            set(btnOdometry, 'Value', 1);
        end
        updateSliderVisibility();
    end

    function updateSliderVisibility()
        isDeadReckoning = get(btnDeadReckoning, 'Value');
        
        if isDeadReckoning
            numSliders = 6;
            set(sliders.alpha5, 'Value', 0.0001);
            set(sliders.alpha6, 'Value', 0.0001);
            set(sliderValues.alpha5, 'String', '0.0001');
            set(sliderValues.alpha6, 'String', '0.0001');
        else
            numSliders = 4;
        end
        
        for i = 1:6
            if i <= numSliders
                set(sliderLabels.(sprintf('alpha%d', i)), 'Visible', 'on');
                set(sliders.(sprintf('alpha%d', i)), 'Visible', 'on');
                set(sliderValues.(sprintf('alpha%d', i)), 'Visible', 'on');
            else
                set(sliderLabels.(sprintf('alpha%d', i)), 'Visible', 'off');
                set(sliders.(sprintf('alpha%d', i)), 'Visible', 'off');
                set(sliderValues.(sprintf('alpha%d', i)), 'Visible', 'off');
            end
        end
    end

    function sliderCallback(src, idx)
        val = get(src, 'Value');
        set(sliderValues.(sprintf('alpha%d', idx)), 'String', sprintf('%.4f', val));
    end

    function liveSimulationCallback(~, ~)
        isLive = get(chkLiveSimulation, 'Value');
        if isLive
            set(btnStop, 'Visible', 'on');
            set(btnContinue, 'Visible', 'on');
            set(btnReset, 'Visible', 'on');
            set(txtPaceLabel, 'Visible', 'on');
            set(editPace, 'Visible', 'on');
        else
            set(btnStop, 'Visible', 'off');
            set(btnContinue, 'Visible', 'off');
            set(btnReset, 'Visible', 'off');
            set(txtPaceLabel, 'Visible', 'off');
            set(editPace, 'Visible', 'off');
        end
    end

    function runSimulation(~, ~)
        set(btnRun, 'Enable', 'off');
        
        cla(ax);
        hold(ax, 'on');
        
        numSamples = str2double(get(editSamples, 'String'));
        if isnan(numSamples) || numSamples < 1
            errordlg('Please enter a valid number of samples (positive integer)');
            set(btnRun, 'Enable', 'on');
            return;
        end
        numSamples = round(numSamples);
        
        isDeadReckoning = get(btnDeadReckoning, 'Value');
        trajectoryType = get(popupTrajectory, 'Value');
        isLive = get(chkLiveSimulation, 'Value');
        
        % Generate trajectory
        switch trajectoryType
            case 1
                [X_gt, U_list, x0, dt] = generateCircularTrajectory();
                occMap = [];
            case 2
                [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
                occMap = [];
            case 3
                [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory();
                show(occMap, 'Parent', ax);
                hold(ax, 'on');
            otherwise
                errordlg('Unknown trajectory type');
                set(btnRun, 'Enable', 'on');
                return;
        end
        
        % Update simulation state
        simState.isRunning = true;
        simState.isPaused = false;
        simState.isFinished = false;
        simState.currentStep = 0;
        simState.X_gt = X_gt;
        simState.U_list = U_list;
        simState.x0 = x0;
        simState.dt = dt;
        simState.numSamples = numSamples;
        simState.isDeadReckoning = isDeadReckoning;
        simState.trajectoryType = trajectoryType;
        simState.occMap = occMap;
        
        if isDeadReckoning
            alpha = zeros(6, 1);
            for i = 1:6
                alpha(i) = get(sliders.(sprintf('alpha%d', i)), 'Value');
            end
            simState.alpha = alpha;
            
            size_U = size(U_list, 2);
            simState.X_samples = repmat(x0, 1, numSamples, size_U+1);
            modelName = 'Dead Reckoning';
        else
            alpha = zeros(4, 1);
            for i = 1:4
                alpha(i) = get(sliders.(sprintf('alpha%d', i)), 'Value');
            end
            simState.alpha = alpha;
            
            % Convert to odometry
            U_odom = [];
            x_odom = x0;
            for i = 1:length(U_list(1,:))
                u_odom = [X_gt(:, i); X_gt(:, i+1)];
                x_odom = GTodometryMotionModel(x_odom, u_odom);
                U_odom = [U_odom, u_odom];
            end
            
            simState.U_odom = U_odom;
            size_U = size(U_odom, 2);
            simState.X_samples = repmat(x0, 1, numSamples, size_U+1);
            modelName = 'Odometry';
        end
        
        if isLive
            set(btnStop, 'Enable', 'on');
            set(btnContinue, 'Enable', 'off');
            
            plot(ax, X_gt(1,1), X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            title(ax, sprintf('%s Model - %s Trajectory - Live Simulation', ...
                                 modelName, trajectoryNames{trajectoryType}));
            legend(ax, 'Location', 'best');
            grid(ax, 'on');
            axis(ax, 'equal');
            
            runLiveSimulation();
        else
            % Batch mode
            if isDeadReckoning
                size_U = size(U_list, 2);
                for i = 1:size_U
                    for sample = 1:numSamples
                        simState.X_samples(:, sample, i+1) = sampleDeadReckoningMotionModel(...
                            simState.X_samples(:, sample, i), U_list(:, i), alpha, dt, occMap);
                    end
                end
            else
                size_U = size(simState.U_odom, 2);
                for i = 1:size_U
                    for sample = 1:numSamples
                        simState.X_samples(:, sample, i+1) = sampleOdometryMotionModel(...
                            simState.X_samples(:, sample, i), simState.U_odom(:, i), alpha, occMap);
                    end
                end
            end
            
            % Plot results
            xpos = simState.X_samples(1, :, :);
            xpos = xpos(:, :);
            ypos = simState.X_samples(2, :, :);
            ypos = ypos(:, :);
            
            plot(ax, xpos, ypos, 'r.', 'MarkerSize', 2, 'DisplayName', 'Samples');
            plot(ax, X_gt(1,:), X_gt(2,:), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            title(ax, sprintf('%s Model - %s Trajectory - %d Samples', ...
                                 modelName, trajectoryNames{trajectoryType}, numSamples));
            legend(ax, 'Location', 'best');
            grid(ax, 'on');
            axis(ax, 'equal');
            
            set(btnRun, 'Enable', 'on');
            set(btnSave, 'Enable', 'on');
        end
    end

    function runLiveSimulation()
        if simState.isFinished
            return;
        end
        
        paceValue = str2double(get(editPace, 'String'));
        if isnan(paceValue) || paceValue < 0
            paceValue = 0.1;
            set(editPace, 'String', '0.1');
        end
        
        if simState.isDeadReckoning
            size_U = size(simState.U_list, 2);
        else
            size_U = size(simState.U_odom, 2);
        end
        
        for step = (simState.currentStep + 1):size_U
            % Check if simulation should stop or pause
            if ~simState.isRunning
                if ~simState.isPaused
                    break;
                else
                    % Paused - exit and wait for continue
                    return;
                end
            end
            
            simState.currentStep = step;
            
            % Propagate samples
            if simState.isDeadReckoning
                for sample = 1:simState.numSamples
                    x_new = sampleDeadReckoningMotionModel(...
                        simState.X_samples(:, sample, step), ...
                        simState.U_list(:, step), ...
                        simState.alpha, simState.dt, simState.occMap);
                    simState.X_samples(:, sample, step+1) = x_new;
                end
            else
                for sample = 1:simState.numSamples
                    x_new = sampleOdometryMotionModel(...
                        simState.X_samples(:, sample, step), ...
                        simState.U_odom(:, step), ...
                        simState.alpha, simState.occMap);
                    simState.X_samples(:, sample, step+1) = x_new;
                end
            end
            
            % Plot
            xpos = simState.X_samples(1, :, 1:(step+1));
            xpos = xpos(:, :);
            ypos = simState.X_samples(2, :, 1:(step+1));
            ypos = ypos(:, :);
            
            children = get(ax, 'Children');
            for i = 1:length(children)
                childType = class(children(i));
                if strcmp(childType, 'matlab.graphics.chart.primitive.Line')
                    markerType = get(children(i), 'Marker');
                    if strcmp(markerType, '.') || strcmp(markerType, 'o')
                        delete(children(i));
                    end
                end
            end
            
            plot(ax, xpos, ypos, 'r.', 'MarkerSize', 2, 'DisplayName', 'Samples');
            plot(ax, simState.X_gt(1, 1:(step+1)), simState.X_gt(2, 1:(step+1)), ...
                 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            if simState.isDeadReckoning
                modelName = 'Dead Reckoning';
            else
                modelName = 'Odometry';
            end
            title(ax, sprintf('%s Model - %s Trajectory - Step %d/%d', ...
                             modelName, trajectoryNames{simState.trajectoryType}, step, size_U));
            
            drawnow;
            pause(paceValue);
        end
        
        % Finished
        simState.isRunning = false;
        simState.isFinished = true;
        set(btnRun, 'Enable', 'on');
        set(btnStop, 'Enable', 'off');
        set(btnContinue, 'Enable', 'off');
        set(btnSave, 'Enable', 'on');
        
        trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
        if simState.isDeadReckoning
            modelName = 'Dead Reckoning';
        else
            modelName = 'Odometry';
        end
        title(ax, sprintf('%s Model - %s Trajectory - Complete (%d samples)', ...
                         modelName, trajectoryNames{simState.trajectoryType}, simState.numSamples));
    end

    function stopSimulation(~, ~)
        simState.isRunning = false;
        simState.isPaused = true;
        set(btnStop, 'Enable', 'off');
        set(btnContinue, 'Enable', 'on');
        set(btnReset, 'Enable', 'on');
        set(btnRun, 'Enable', 'off');
    end

    function continueSimulation(~, ~)
        if ~isfield(simState, 'X_samples') || isempty(simState.X_samples)
            errordlg('No simulation to continue. Please run a simulation first.');
            return;
        end
        
        simState.isRunning = true;
        simState.isPaused = false;
        set(btnStop, 'Enable', 'on');
        set(btnContinue, 'Enable', 'off');
        set(btnReset, 'Enable', 'off');
        set(btnRun, 'Enable', 'off');
        
        runLiveSimulation();
    end

    function resetSimulation(~, ~)
        if ~isfield(simState, 'X_samples') || isempty(simState.X_samples)
            errordlg('No simulation to reset. Please run a simulation first.');
            return;
        end
        
        simState.isRunning = false;
        simState.isPaused = false;
        simState.isFinished = false;
        simState.currentStep = 0;
        
        % Reset samples
        if simState.isDeadReckoning
            size_U = size(simState.U_list, 2);
            simState.X_samples = repmat(simState.x0, 1, simState.numSamples, size_U+1);
        else
            size_U = size(simState.U_odom, 2);
            simState.X_samples = repmat(simState.x0, 1, simState.numSamples, size_U+1);
        end
        
        % Clear plot
        cla(ax);
        hold(ax, 'on');
        
        if ~isempty(simState.occMap)
            show(simState.occMap, 'Parent', ax);
            hold(ax, 'on');
        end
        
        plot(ax, simState.X_gt(1,1), simState.X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
        
        trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
        if simState.isDeadReckoning
            modelName = 'Dead Reckoning';
        else
            modelName = 'Odometry';
        end
        title(ax, sprintf('%s Model - %s Trajectory - Reset', ...
                         modelName, trajectoryNames{simState.trajectoryType}));
        legend(ax, 'Location', 'best');
        grid(ax, 'on');
        axis(ax, 'equal');
        
        set(btnRun, 'Enable', 'on');
        set(btnStop, 'Enable', 'off');
        set(btnContinue, 'Enable', 'off');
        set(btnReset, 'Enable', 'off');
    end

    function saveResults(~, ~)
        if ~isfield(simState, 'X_samples') || isempty(simState.X_samples)
            errordlg('No simulation data to save. Please run a simulation first.');
            return;
        end
        
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        trajectoryNames = {'Circular', 'Rectangle', 'RectangleObstacles'};
        trajectoryName = trajectoryNames{simState.trajectoryType};
        
        if simState.isDeadReckoning
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
            % Save figure
            figureFilename = fullfile(pathname, [filenameNoExt, '.png']);
            
            wasLiveSimChecked = get(chkLiveSimulation, 'Value');
            
            % Temporarily hide all UI components
            allComponents = panel.components;
            set(allComponents, 'Visible', 'off');
            
            exportgraphics(fig, figureFilename, 'Resolution', 300);
            
            % Restore visibility
            set(allComponents, 'Visible', 'on');
            updateSliderVisibility();
            
            if wasLiveSimChecked
                set(btnStop, 'Visible', 'on');
                set(btnContinue, 'Visible', 'on');
                set(btnReset, 'Visible', 'on');
                set(txtPaceLabel, 'Visible', 'on');
                set(editPace, 'Visible', 'on');
            else
                set(btnStop, 'Visible', 'off');
                set(btnContinue, 'Visible', 'off');
                set(btnReset, 'Visible', 'off');
                set(txtPaceLabel, 'Visible', 'off');
                set(editPace, 'Visible', 'off');
            end
            
            % Save MAT file
            matFilename = fullfile(pathname, [filenameNoExt, '.mat']);
            simulationData = struct();
            simulationData.X_samples = simState.X_samples;
            simulationData.X_gt = simState.X_gt;
            simulationData.U_list = simState.U_list;
            simulationData.x0 = simState.x0;
            simulationData.dt = simState.dt;
            simulationData.alpha = simState.alpha;
            simulationData.numSamples = simState.numSamples;
            simulationData.modelType = modelName;
            simulationData.trajectoryType = trajectoryName;
            simulationData.timestamp = timestamp;
            
            if ~simState.isDeadReckoning
                simulationData.U_odom = simState.U_odom;
            end
            
            save(matFilename, 'simulationData');
            
            % Save config TXT
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
            fprintf(fid, 'Number of Samples: %d\n', simState.numSamples);
            fprintf(fid, 'Time Step (dt): %.4f seconds\n', simState.dt);
            fprintf(fid, 'Number of Control Steps: %d\n', size(simState.U_list, 2));
            fprintf(fid, '\n');
            
            fprintf(fid, 'NOISE PARAMETERS (Alpha)\n');
            fprintf(fid, '-------------------------\n');
            if simState.isDeadReckoning
                fprintf(fid, 'α1 (v→v noise): %.6f\n', simState.alpha(1));
                fprintf(fid, 'α2 (ω→v noise): %.6f\n', simState.alpha(2));
                fprintf(fid, 'α3 (v→ω noise): %.6f\n', simState.alpha(3));
                fprintf(fid, 'α4 (ω→ω noise): %.6f\n', simState.alpha(4));
                fprintf(fid, 'α5 (v→γ noise): %.6f\n', simState.alpha(5));
                fprintf(fid, 'α6 (ω→γ noise): %.6f\n', simState.alpha(6));
            else
                fprintf(fid, 'α1 (rot→rot noise): %.6f\n', simState.alpha(1));
                fprintf(fid, 'α2 (trans→rot noise): %.6f\n', simState.alpha(2));
                fprintf(fid, 'α3 (trans→trans noise): %.6f\n', simState.alpha(3));
                fprintf(fid, 'α4 (rot→trans noise): %.6f\n', simState.alpha(4));
            end
            fprintf(fid, '\n');
            
            fprintf(fid, 'INITIAL POSE\n');
            fprintf(fid, '------------\n');
            fprintf(fid, 'x: %.4f m\n', simState.x0(1));
            fprintf(fid, 'y: %.4f m\n', simState.x0(2));
            fprintf(fid, 'θ: %.4f rad (%.2f deg)\n', simState.x0(3), rad2deg(simState.x0(3)));
            fprintf(fid, '\n');
            
            fprintf(fid, 'FINAL GROUND TRUTH POSE\n');
            fprintf(fid, '-----------------------\n');
            finalGT = simState.X_gt(:, end);
            fprintf(fid, 'x: %.4f m\n', finalGT(1));
            fprintf(fid, 'y: %.4f m\n', finalGT(2));
            fprintf(fid, 'θ: %.4f rad (%.2f deg)\n', finalGT(3), rad2deg(finalGT(3)));
            fprintf(fid, '\n');
            
            if simState.trajectoryType == 3
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

end  % End of createModelPanel function

%% Trajectory Generation Functions (outside createModelPanel)

function [X_gt, U_list, x0, dt] = generateCircularTrajectory()
    x0 = [0; 0; -pi/2];
    v = 1;
    r = 1;
    w = -v/r;
    dt = abs((pi/4)/w);
    u = [v; w];
    
    x_gt = x0;
    X_gt = x_gt;
    U_list = [];
    
    for i = 1:8
        x_gt = GTdeadReckoningMotionModel(x_gt, u, dt);
        X_gt = [X_gt, x_gt];
        U_list = [U_list, u];
    end
end

function [X_gt, U_list, x0, dt] = generateRectangleTrajectory()
    x0 = [0; 0; 0];
    X_gt = x0;
    U_list = [];
    x = x0;
    
    % Go straight
    v = 2; w = 0; dt = 1; u = [v; w];
    for i = 1:4
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Turn
    v = 0; w = pi/2; dt = 1; u = [v; w];
    x = GTdeadReckoningMotionModel(x, u, dt);
    U_list = [U_list, u];
    X_gt = [X_gt, x];
    
    % Go straight
    v = 2; w = 0; dt = 1; u = [v; w];
    for i = 1:4
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Turn
    v = 0; w = pi/2; dt = 1; u = [v; w];
    x = GTdeadReckoningMotionModel(x, u, dt);
    U_list = [U_list, u];
    X_gt = [X_gt, x];
    
    % Go straight
    v = 2; w = 0; dt = 1; u = [v; w];
    for i = 1:4
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
end

function [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory()
    [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
    occMap = createOccupancyMapWithObstacles();
end
