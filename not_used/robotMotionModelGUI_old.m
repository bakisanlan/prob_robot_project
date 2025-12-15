function robotMotionModelGUI()
    %ROBOTMOTIONMODELGUI Interactive GUI for robot motion model visualization
    close all; clc; clear

    addpath("functions")
   
    % Get screen size
    screenSize = get(0, 'ScreenSize');
    screenWidth = screenSize(3);
    screenHeight = screenSize(4);
    
    % Define desired figure size (1200x700) as reference
    figWidth = min(1200, screenWidth * 0.9);  % Max 90% of screen width
    figHeight = min(700, screenHeight * 0.85); % Max 85% of screen height
    
    % Maintain aspect ratio if needed
    aspectRatio = 1200 / 700;
    if figWidth / figHeight > aspectRatio
        figWidth = figHeight * aspectRatio;
    else
        figHeight = figWidth / aspectRatio;
    end
    
    % Center the figure on screen
    figX = (screenWidth - figWidth) / 2;
    figY = (screenHeight - figHeight) / 2;
    
    % Create figure with normalized units
    fig = figure('Name', 'Robot Motion Model Simulator', ...
                 'NumberTitle', 'off', ...
                 'Units', 'pixels', ...
                 'Position', [figX, figY, figWidth, figHeight], ...
                 'Resize', 'on', ...
                 'WindowState', 'maximized', ...
                 'SizeChangedFcn', @resizeCallback);
    
    % Calculate relative dimensions (based on 1200x700 reference)
    plotWidth = figWidth * 0.667;   % 800/1200 = 66.7%
    plotHeight = figHeight * 0.857;  % 600/700 = 85.7%
    plotX = figWidth * 0.042;        % 50/1200
    plotY = figHeight * 0.071;       % 50/700
    
    panelX = figWidth * 0.75;        % 900/1200
    panelWidth = figWidth * 0.233;   % 280/1200
    
    % Create main axes for plotting (70% of width, full height minus margin)
    ax = axes('Parent', fig, ...
              'Units', 'pixels', ...
              'Position', [plotX, plotY, plotWidth, plotHeight]);
    axis equal;
    grid on;
    hold(ax, 'on');
    title(ax, 'Robot Pose Samples');
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');
    
    % Create panel selection buttons at the top
    btnWidth = panelWidth / 3 - 5;
    btnHeight = figHeight * 0.05;
    btnY = figHeight * 0.929;  % 650/700
    
    btnPanelModel = uicontrol('Style', 'pushbutton', ...
                              'String', 'MODEL', ...
                              'Position', [panelX, btnY, btnWidth, btnHeight], ...
                              'FontSize', 10, ...
                              'FontWeight', 'bold', ...
                              'BackgroundColor', [0.3, 0.6, 0.9], ...
                              'Callback', @(~,~)switchPanel('MODEL'));
    
    btnPanelSensors = uicontrol('Style', 'pushbutton', ...
                                'String', 'SENSORS', ...
                                'Position', [panelX+btnWidth+5, btnY, btnWidth, btnHeight], ...
                                'FontSize', 10, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.7, 0.7, 0.7], ...
                                'Callback', @(~,~)switchPanel('SENSORS'));
    
    btnPanelGNC = uicontrol('Style', 'pushbutton', ...
                            'String', 'GNC', ...
                            'Position', [panelX+2*btnWidth+10, btnY, btnWidth, btnHeight], ...
                            'FontSize', 10, ...
                            'FontWeight', 'bold', ...
                            'BackgroundColor', [0.7, 0.7, 0.7], ...
                            'Callback', @(~,~)switchPanel('GNC'));
    
    % Separator line
    uicontrol('Style', 'text', ...
              'String', '', ...
              'Position', [panelX, btnY-5, panelWidth, 2], ...
              'BackgroundColor', [0.3, 0.3, 0.3]);
    
    %% MODEL PANEL COMPONENTS
    % Model selection buttons
    txtModelLabel = uicontrol('Style', 'text', ...
              'String', 'Motion Model:', ...
              'Position', [panelX, figHeight*0.857, panelWidth*0.6, figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    btnDeadReckoning = uicontrol('Style', 'radiobutton', ...
                                  'String', 'Dead Reckoning', ...
                                  'Position', [panelX, figHeight*0.814, panelWidth*0.5, figHeight*0.036], ...
                                  'Value', 1, ...
                                  'Callback', @modelSelectionCallback);
    
    btnOdometry = uicontrol('Style', 'radiobutton', ...
                            'String', 'Odometry', ...
                            'Position', [panelX, figHeight*0.771, panelWidth*0.5, figHeight*0.036], ...
                            'Value', 0, ...
                            'Callback', @modelSelectionCallback);
    
    % Trajectory selection
    txtTrajectoryLabel = uicontrol('Style', 'text', ...
              'String', 'Trajectory Type:', ...
              'Position', [panelX, figHeight*0.72, panelWidth*0.6, figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    popupTrajectory = uicontrol('Style', 'popupmenu', ...
                                'String', {'Circular', 'Rectangle', 'Rectangle with Obstacles'}, ...
                                'Position', [panelX, figHeight*0.684, panelWidth*0.7, figHeight*0.036], ...
                                'Value', 1);
    
    % Number of samples input
    txtSamplesLabel = uicontrol('Style', 'text', ...
              'String', 'Number of Samples:', ...
              'Position', [panelX, figHeight*0.63, panelWidth*0.6, figHeight*0.036], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, 'FontWeight', 'bold');
    
    editSamples = uicontrol('Style', 'edit', ...
                            'String', '1000', ...
                            'Position', [panelX, figHeight*0.594, panelWidth*0.4, figHeight*0.036], ...
                            'HorizontalAlignment', 'center');
    
    % Alpha parameters label
    txtAlphaLabel = uicontrol('Style', 'text', ...
                              'String', 'Alpha Parameters:', ...
                              'Position', [panelX, figHeight*0.53, panelWidth*0.6, figHeight*0.036], ...
                              'HorizontalAlignment', 'left', ...
                              'FontSize', 10, 'FontWeight', 'bold');
    
    % Create 6 sliders (show/hide based on model)
    sliders = struct();
    sliderLabels = struct();
    sliderValues = struct();
    
    sliderSpacing = figHeight * 0.046;  % 60/700
    sliderHeight = figHeight * 0.029;   % 20/700
    
    for i = 1:6
        yPos = figHeight*0.5 - (i-1)*sliderSpacing;
        
        sliderLabels.(sprintf('alpha%d', i)) = uicontrol('Style', 'text', ...
            'String', sprintf('α%d:', i), ...
            'Position', [panelX, yPos, panelWidth*0.12, sliderHeight], ...
            'HorizontalAlignment', 'left', ...
            'Tag', sprintf('sliderLabel%d', i));  % Add tag for identification
        
        sliders.(sprintf('alpha%d', i)) = uicontrol('Style', 'slider', ...
            'Min', 0, 'Max', 0.01, 'Value', 0.001, ...
            'Position', [panelX+panelWidth*0.14, yPos, panelWidth*0.64, sliderHeight], ...
            'Callback', {@sliderCallback, i}, ...
            'Tag', sprintf('slider%d', i));  % Add tag for identification
        
        sliderValues.(sprintf('alpha%d', i)) = uicontrol('Style', 'text', ...
            'String', '0.001', ...
            'Position', [panelX+panelWidth*0.79, yPos, panelWidth*0.18, sliderHeight], ...
            'HorizontalAlignment', 'left', ...
            'Tag', sprintf('sliderValue%d', i));  % Add tag for identification
    end
    
    % Live simulation checkbox
    chkLiveSimulation = uicontrol('Style', 'checkbox', ...
                                  'String', 'Live Simulation', ...
                                  'Position', [panelX, figHeight*0.15, panelWidth*0.5, figHeight*0.036], ...
                                  'Value', 0, ...
                                  'FontSize', 10, ...
                                  'Callback', @liveSimulationCallback);
    
    % Pace label and input (initially hidden)
    txtPaceLabel = uicontrol('Style', 'text', ...
                             'String', 'Pace (s):', ...
                             'Position', [panelX, figHeight*0.11, panelWidth*0.3, figHeight*0.03], ...
                             'HorizontalAlignment', 'left', ...
                             'FontSize', 9, ...
                             'Visible', 'off');
    
    editPace = uicontrol('Style', 'edit', ...
                         'String', '0.1', ...
                         'Position', [panelX+panelWidth*0.32, figHeight*0.11, panelWidth*0.25, figHeight*0.03], ...
                         'HorizontalAlignment', 'center', ...
                         'FontSize', 9, ...
                         'Visible', 'off');
    
    % Simulation control buttons
    btnRunModel = uicontrol('Style', 'pushbutton', ...
                       'String', 'Run', ...
                       'Position', [panelX, figHeight*0.06, panelWidth*0.22, figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.4, 0.7, 0.4], ...
                       'Callback', @runSimulation);
    
    btnStopModel = uicontrol('Style', 'pushbutton', ...
                       'String', 'Stop', ...
                       'Position', [panelX+panelWidth*0.25, figHeight*0.06, panelWidth*0.22, figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.9, 0.4, 0.4], ...
                       'Enable', 'off', ...
                       'Visible', 'off', ...
                       'Callback', @stopSimulation);
    
    btnContinueModel = uicontrol('Style', 'pushbutton', ...
                       'String', 'Continue', ...
                       'Position', [panelX+panelWidth*0.5, figHeight*0.06, panelWidth*0.22, figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.4, 0.6, 0.9], ...
                       'Enable', 'off', ...
                       'Visible', 'off', ...
                       'Callback', @continueSimulation);
    
    btnResetModel = uicontrol('Style', 'pushbutton', ...
                       'String', 'Reset', ...
                       'Position', [panelX+panelWidth*0.75, figHeight*0.06, panelWidth*0.22, figHeight*0.045], ...
                       'FontSize', 11, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.9, 0.7, 0.2], ...
                       'Enable', 'off', ...
                       'Visible', 'off', ...
                       'Callback', @resetSimulation);
    
    % Save button
    btnSaveResults = uicontrol('Style', 'pushbutton', ...
                       'String', 'Save Results', ...
                       'Position', [panelX, figHeight*0.01, panelWidth*0.5, figHeight*0.04], ...
                       'FontSize', 10, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0.3, 0.8, 0.8], ...
                       'Enable', 'off', ...
                       'Callback', @saveResults);
    
    % Store all MODEL panel components (properly flatten cell arrays)
    sliderLabelsList = struct2cell(sliderLabels);
    slidersList = struct2cell(sliders);
    sliderValuesList = struct2cell(sliderValues);
    
    modelPanelComponents = [txtModelLabel, btnDeadReckoning, btnOdometry, ...
                            txtTrajectoryLabel, popupTrajectory, ...
                            txtSamplesLabel, editSamples, txtAlphaLabel, ...
                            chkLiveSimulation, txtPaceLabel, editPace, ...
                            btnRunModel, btnStopModel, btnContinueModel, btnResetModel, btnSaveResults, ...
                            sliderLabelsList{:}, slidersList{:}, sliderValuesList{:}];
    
    %% SENSORS PANEL COMPONENTS
    txtSensorsTitle = uicontrol('Style', 'text', ...
              'String', 'Sensors Panel', ...
              'Position', [panelX, figHeight*0.857, panelWidth*0.75, figHeight*0.043], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 12, 'FontWeight', 'bold', ...
              'Visible', 'off');
    
    txtSensorsPlaceholder = uicontrol('Style', 'text', ...
              'String', 'Sensor configuration controls will be added here...', ...
              'Position', [panelX, figHeight*0.429, panelWidth, figHeight*0.071], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 10, ...
              'Visible', 'off');
    
    % Store all SENSORS panel components
    sensorsPanelComponents = [txtSensorsTitle, txtSensorsPlaceholder];
    
    %% GNC PANEL COMPONENTS
    txtGNCTitle = uicontrol('Style', 'text', ...
              'String', 'GNC Panel', ...
              'Position', [panelX, figHeight*0.857, panelWidth*0.75, figHeight*0.043], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 12, 'FontWeight', 'bold', ...
              'Visible', 'off');
    
    txtGNCPlaceholder = uicontrol('Style', 'text', ...
              'String', 'Guidance, Navigation, and Control options will be added here...', ...
              'Position', [panelX, figHeight*0.429, panelWidth, figHeight*0.071], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 10, ...
              'Visible', 'off');
    
    % Store all GNC panel components
    gncPanelComponents = [txtGNCTitle, txtGNCPlaceholder];
    
    % Simulation state variables (declare before callback functions)
    simState = struct();
    simState.isRunning = false;
    simState.isPaused = false;
    simState.currentStep = 0;
    simState.X_samples = [];
    simState.X_gt = [];
    simState.U_list = [];
    simState.U_odom = [];
    simState.x0 = [];
    simState.dt = 0;
    simState.numSamples = 0;
    simState.alpha = [];
    simState.isDeadReckoning = true;
    simState.trajectoryType = 1;
    simState.occMap = [];  % Add occupancy map to state
    
    % Initialize with MODEL panel visible
    currentPanel = 'MODEL';
    updateSliderVisibility();
    
    %% Resize Callback Function
    
    function resizeCallback(src, ~)
        % Check if figure still exists and is valid
        if ~isvalid(src)
            return;
        end
        
        % Try-catch to prevent errors during resize
        try
            % Get new figure size
            figPos = get(src, 'Position');
            newWidth = figPos(3);
            newHeight = figPos(4);
            
            % Recalculate dimensions based on new size
            newPlotWidth = newWidth * 0.667;
            newPlotHeight = newHeight * 0.857;
            newPlotX = newWidth * 0.042;
            newPlotY = newHeight * 0.071;
            
            newPanelX = newWidth * 0.75;
            newPanelWidth = newWidth * 0.233;
            
            % Find and update axes position
            axHandle = findobj(src, 'Type', 'axes');
            if ~isempty(axHandle)
                set(axHandle(1), 'Position', [newPlotX, newPlotY, newPlotWidth, newPlotHeight]);
            end
            
            % Update panel selection buttons
            newBtnWidth = newPanelWidth / 3 - 5;
            newBtnHeight = figHeight * 0.05;
            newBtnY = newHeight * 0.929;
            
            % Find and update panel buttons
            btnModel = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'MODEL');
            btnSensors = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'SENSORS');
            btnGNC = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'GNC');
            
            if ~isempty(btnModel)
                set(btnModel, 'Position', [newPanelX, newBtnY, newBtnWidth, newBtnHeight]);
            end
            if ~isempty(btnSensors)
                set(btnSensors, 'Position', [newPanelX+newBtnWidth+5, newBtnY, newBtnWidth, newBtnHeight]);
            end
            if ~isempty(btnGNC)
                set(btnGNC, 'Position', [newPanelX+2*newBtnWidth+10, newBtnY, newBtnWidth, newBtnHeight]);
            end
            
            % Update separator line
            separatorHandle = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', '');
            if ~isempty(separatorHandle) && length(separatorHandle) >= 1
                set(separatorHandle(1), 'Position', [newPanelX, newBtnY-5, newPanelWidth, 2]);
            end
            
            % Update MODEL panel components
            txtModelLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Motion Model:');
            if ~isempty(txtModelLabel)
                set(txtModelLabel, 'Position', [newPanelX, newHeight*0.857, newPanelWidth*0.6, newHeight*0.036]);
            end
            
            btnDR = findobj(src, 'Type', 'uicontrol', 'Style', 'radiobutton', 'String', 'Dead Reckoning');
            if ~isempty(btnDR)
                set(btnDR, 'Position', [newPanelX, newHeight*0.814, newPanelWidth*0.5, newHeight*0.036]);
            end
            
            btnOdom = findobj(src, 'Type', 'uicontrol', 'Style', 'radiobutton', 'String', 'Odometry');
            if ~isempty(btnOdom)
                set(btnOdom, 'Position', [newPanelX, newHeight*0.771, newPanelWidth*0.5, newHeight*0.036]);
            end
            
            txtTrajLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Trajectory Type:');
            if ~isempty(txtTrajLabel)
                set(txtTrajLabel, 'Position', [newPanelX, newHeight*0.72, newPanelWidth*0.6, newHeight*0.036]);
            end
            
            popupTraj = findobj(src, 'Type', 'uicontrol', 'Style', 'popupmenu');
            if ~isempty(popupTraj)
                set(popupTraj(1), 'Position', [newPanelX, newHeight*0.684, newPanelWidth*0.7, newHeight*0.036]);
            end
            
            txtSamplesLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Number of Samples:');
            if ~isempty(txtSamplesLabel)
                set(txtSamplesLabel, 'Position', [newPanelX, newHeight*0.63, newPanelWidth*0.6, newHeight*0.036]);
            end
            
            editSamp = findobj(src, 'Type', 'uicontrol', 'Style', 'edit');
            for i = 1:length(editSamp)
                str = get(editSamp(i), 'String');
                numVal = str2double(str);
                if ~isnan(numVal) && numVal > 10  % Likely the samples field
                    set(editSamp(i), 'Position', [newPanelX, newHeight*0.594, newPanelWidth*0.4, newHeight*0.036]);
                    break;
                end
            end
            
            txtAlphaLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Alpha Parameters:');
            if ~isempty(txtAlphaLabel)
                set(txtAlphaLabel, 'Position', [newPanelX, newHeight*0.53, newPanelWidth*0.6, newHeight*0.036]);
            end
            
            % Update sliders using Tags for reliable identification
            newSliderSpacing = newHeight * 0.046;
            newSliderHeight = newHeight * 0.029;
            
            % Update each slider, label, and value by index (1-6)
            for i = 1:6
                yPos = newHeight*0.5 - (i-1)*newSliderSpacing;
                
                % Find components by their unique tags
                sliderLabel = findobj(src, 'Tag', sprintf('sliderLabel%d', i));
                slider = findobj(src, 'Tag', sprintf('slider%d', i));
                sliderValue = findobj(src, 'Tag', sprintf('sliderValue%d', i));
                
                % Update positions
                if ~isempty(sliderLabel)
                    set(sliderLabel, 'Position', [newPanelX, yPos, newPanelWidth*0.12, newSliderHeight]);
                end
                
                if ~isempty(slider)
                    set(slider, 'Position', [newPanelX+newPanelWidth*0.14, yPos, newPanelWidth*0.64, newSliderHeight]);
                end
                
                if ~isempty(sliderValue)
                    set(sliderValue, 'Position', [newPanelX+newPanelWidth*0.79, yPos, newPanelWidth*0.18, newSliderHeight]);
                end
            end
            
            % Update live simulation controls
            chkLive = findobj(src, 'Type', 'uicontrol', 'Style', 'checkbox', 'String', 'Live Simulation');
            if ~isempty(chkLive)
                set(chkLive, 'Position', [newPanelX, newHeight*0.15, newPanelWidth*0.5, newHeight*0.036]);
            end
            
            txtPace = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Pace (s):');
            if ~isempty(txtPace)
                set(txtPace, 'Position', [newPanelX, newHeight*0.11, newPanelWidth*0.3, newHeight*0.03]);
            end
            
            % Update pace edit field
            editPaceFields = findobj(src, 'Type', 'uicontrol', 'Style', 'edit');
            for i = 1:length(editPaceFields)
                str = get(editPaceFields(i), 'String');
                numVal = str2double(str);
                if ~isnan(numVal) && numVal >= 0 && numVal < 10  % Likely the pace field (0.1)
                    set(editPaceFields(i), 'Position', [newPanelX+newPanelWidth*0.32, newHeight*0.11, newPanelWidth*0.25, newHeight*0.03]);
                    break;
                end
            end
            
            % Update simulation control buttons
            btnRun = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Run');
            if ~isempty(btnRun)
                set(btnRun, 'Position', [newPanelX, newHeight*0.06, newPanelWidth*0.22, newHeight*0.045]);
            end
            
            btnStop = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Stop');
            if ~isempty(btnStop)
                set(btnStop, 'Position', [newPanelX+newPanelWidth*0.25, newHeight*0.06, newPanelWidth*0.22, newHeight*0.045]);
            end
            
            btnCont = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Continue');
            if ~isempty(btnCont)
                set(btnCont, 'Position', [newPanelX+newPanelWidth*0.5, newHeight*0.06, newPanelWidth*0.22, newHeight*0.045]);
            end
            
            btnReset = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Reset');
            if ~isempty(btnReset)
                set(btnReset, 'Position', [newPanelX+newPanelWidth*0.75, newHeight*0.06, newPanelWidth*0.22, newHeight*0.045]);
            end
            
            % Update save button
            btnSave = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Save Results');
            if ~isempty(btnSave)
                set(btnSave, 'Position', [newPanelX, newHeight*0.01, newPanelWidth*0.5, newHeight*0.04]);
            end
            
            % Update SENSORS panel components
            txtSensorsTitle = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Sensors Panel');
            if ~isempty(txtSensorsTitle)
                set(txtSensorsTitle, 'Position', [newPanelX, newHeight*0.857, newPanelWidth*0.75, newHeight*0.043]);
            end
            
            txtSensorsPlaceholder = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Sensor configuration controls will be added here...');
            if ~isempty(txtSensorsPlaceholder)
                set(txtSensorsPlaceholder, 'Position', [newPanelX, newHeight*0.429, newPanelWidth, newHeight*0.071]);
            end
            
            % Update GNC panel components
            txtGNCTitle = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'GNC Panel');
            if ~isempty(txtGNCTitle)
                set(txtGNCTitle, 'Position', [newPanelX, newHeight*0.857, newPanelWidth*0.75, newHeight*0.043]);
            end
            
            txtGNCPlaceholder = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Guidance, Navigation, and Control options will be added here...');
            if ~isempty(txtGNCPlaceholder)
                set(txtGNCPlaceholder, 'Position', [newPanelX, newHeight*0.429, newPanelWidth, newHeight*0.071]);
            end
            
        catch ME
            % Silently catch errors during resize to prevent disruption
            % Optionally log the error: disp(ME.message);
        end
    end
    
    %% Callback Functions
    
    function switchPanel(panelName)
        % Hide all panels
        set(modelPanelComponents, 'Visible', 'off');
        set(sensorsPanelComponents, 'Visible', 'off');
        set(gncPanelComponents, 'Visible', 'off');
        
        % Reset button colors
        set(btnPanelModel, 'BackgroundColor', [0.7, 0.7, 0.7]);
        set(btnPanelSensors, 'BackgroundColor', [0.7, 0.7, 0.7]);
        set(btnPanelGNC, 'BackgroundColor', [0.7, 0.7, 0.7]);
        
        % Show selected panel and highlight button
        currentPanel = panelName;
        switch panelName
            case 'MODEL'
                set(modelPanelComponents, 'Visible', 'on');
                set(btnPanelModel, 'BackgroundColor', [0.3, 0.6, 0.9]);
                updateSliderVisibility();
            case 'SENSORS'
                set(sensorsPanelComponents, 'Visible', 'on');
                set(btnPanelSensors, 'BackgroundColor', [0.3, 0.6, 0.9]);
            case 'GNC'
                set(gncPanelComponents, 'Visible', 'on');
                set(btnPanelGNC, 'BackgroundColor', [0.3, 0.6, 0.9]);
        end
    end
    
    function modelSelectionCallback(src, ~)
        % Toggle radio buttons
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
        % Show/hide sliders based on model
        isDeadReckoning = get(btnDeadReckoning, 'Value');
        
        if isDeadReckoning
            % Dead reckoning: show 6 sliders
            numSliders = 6;
            set(sliders.alpha5, 'Value', 0.0001);
            set(sliders.alpha6, 'Value', 0.0001);
            set(sliderValues.alpha5, 'String', '0.0001');
            set(sliderValues.alpha6, 'String', '0.0001');
        else
            % Odometry: show 4 sliders
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
    
    function sliderCallback(src, ~, idx)
        % Update slider value display
        val = get(src, 'Value');
        set(sliderValues.(sprintf('alpha%d', idx)), 'String', sprintf('%.4f', val));
    end
    
    function liveSimulationCallback(~, ~)
        % Update button visibility based on live simulation checkbox
        isLive = get(chkLiveSimulation, 'Value');
        if isLive
            set(btnStopModel, 'Visible', 'on');
            set(btnContinueModel, 'Visible', 'on');
            set(btnResetModel, 'Visible', 'on');
            set(txtPaceLabel, 'Visible', 'on');
            set(editPace, 'Visible', 'on');
        else
            set(btnStopModel, 'Visible', 'off');
            set(btnContinueModel, 'Visible', 'off');
            set(btnResetModel, 'Visible', 'off');
            set(txtPaceLabel, 'Visible', 'off');
            set(editPace, 'Visible', 'off');
        end
    end
    
    function runSimulation(~, ~)
        % Disable run button during simulation
        set(btnRunModel, 'Enable', 'off');
        
        % Clear previous plot
        cla(ax);
        hold(ax, 'on');
        
        % Get parameters
        numSamples = str2double(get(editSamples, 'String'));
        if isnan(numSamples) || numSamples < 1
            errordlg('Please enter a valid number of samples (positive integer)');
            set(btnRunModel, 'Enable', 'on');
            return;
        end
        numSamples = round(numSamples);
        
        isDeadReckoning = get(btnDeadReckoning, 'Value');
        trajectoryType = get(popupTrajectory, 'Value');
        isLive = get(chkLiveSimulation, 'Value');
        
        % Generate trajectory based on selection
        switch trajectoryType
            case 1  % Circular trajectory
                [X_gt, U_list, x0, dt] = generateCircularTrajectory();
                simState.occMap = [];  % No obstacles
            case 2  % Rectangle trajectory
                [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
                simState.occMap = [];  % No obstacles
            case 3  % Rectangle with obstacles
                [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory();
                simState.occMap = occMap;
                % Visualize occupancy map
                show(occMap, 'Parent', ax);
                hold(ax, 'on');
            otherwise
                errordlg('Unknown trajectory type');
                set(btnRunModel, 'Enable', 'on');
                return;
        end
        
        % Update simulation state (use struct assignment to modify the shared variable)
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
        
        if isDeadReckoning
            % Dead Reckoning Model
            alpha = zeros(6, 1);
            for i = 1:6
                alpha(i) = get(sliders.(sprintf('alpha%d', i)), 'Value');
            end
            simState.alpha = alpha;
            
            size_U = size(U_list, 2);
            simState.X_samples = repmat(x0, 1, numSamples, size_U+1);
            
            modelName = 'Dead Reckoning';
        else
            % Odometry Model
            alpha = zeros(4, 1);
            for i = 1:4
                alpha(i) = get(sliders.(sprintf('alpha%d', i)), 'Value');
            end
            simState.alpha = alpha;
            
            % Convert to odometry control format
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
            % Live simulation mode
            set(btnStopModel, 'Enable', 'on');
            set(btnContinueModel, 'Enable', 'off');
            
            % Plot initial ground truth point only
            plot(ax, X_gt(1,1), X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
            
            trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
            title(ax, sprintf('%s Model - %s Trajectory - Live Simulation', ...
                             modelName, trajectoryNames{trajectoryType}));
            legend(ax, 'Location', 'best');
            grid(ax, 'on');
            axis(ax, 'equal');
            
            % Run live simulation
            runLiveSimulation();
        else
            % Batch mode - run all at once
            if isDeadReckoning
                size_U = size(U_list, 2);
                for i = 1:size_U
                    for sample = 1:numSamples
                        simState.X_samples(:, sample, i+1) = sampleDeadReckoningMotionModel(...
                            simState.X_samples(:, sample, i), U_list(:, i), alpha, dt, simState.occMap);
                    end
                end
            else
                size_U = size(simState.U_odom, 2);
                for i = 1:size_U
                    for sample = 1:numSamples
                        simState.X_samples(:, sample, i+1) = sampleOdometryMotionModel(...
                            simState.X_samples(:, sample, i), simState.U_odom(:, i), alpha, simState.occMap);
                    end
                end
            end
            
            % Plot all results
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
            
            % Re-enable run button
            set(btnRunModel, 'Enable', 'on');
            
            % Enable save button after successful simulation
            set(btnSaveResults, 'Enable', 'on');
        end
    end
    
    function runLiveSimulation()

        if ~simState.isFinished
            
            % Get pace from user input
            paceValue = str2double(get(editPace, 'String'));
            if isnan(paceValue) || paceValue < 0
                paceValue = 0.1;  % Default to 0.1s if invalid input
                set(editPace, 'String', '0.1');
            end
                       
            % Live simulation loop
            if simState.isDeadReckoning
                size_U = size(simState.U_list, 2);
            else
                size_U = size(simState.U_odom, 2);
            end
            
            for step = (simState.currentStep + 1):size_U
                if ~simState.isRunning
                    if ~simState.isPaused
                        break;
                    else
                        while simState.isPaused
                            return;
                        end
                    end
                end
                
                simState.currentStep = step;
                
                % Propagate samples for this time step
                if simState.isDeadReckoning
                    for sample = 1:simState.numSamples
                        x_new = sampleDeadReckoningMotionModel(...
                            simState.X_samples(:, sample, step), ...
                            simState.U_list(:, step), ...
                            simState.alpha, simState.dt, simState.occMap);
                        
                        % Store result (NaN if collision, valid pose otherwise)
                        simState.X_samples(:, sample, step+1) = x_new;
                    end
                else
                    for sample = 1:simState.numSamples
                        x_new = sampleOdometryMotionModel(...
                            simState.X_samples(:, sample, step), ...
                            simState.U_odom(:, step), ...
                            simState.alpha, simState.occMap);
                        
                        % Store result (NaN if collision, valid pose otherwise)
                        simState.X_samples(:, sample, step+1) = x_new;
                    end
                end
                
                % Plot current samples (all time steps up to current)
                xpos = simState.X_samples(1, :, 1:(step+1));
                xpos = xpos(:, :);
                ypos = simState.X_samples(2, :, 1:(step+1));
                ypos = ypos(:, :);
                
                % Clear only sample points and previous ground truth markers
                % Keep occupancy map (Image object) and legend
                children = get(ax, 'Children');
                for i = 1:length(children)
                    childType = class(children(i));
                    % Only delete Line objects (samples and ground truth markers)
                    % Skip Image objects (occupancy map) and other objects
                    if strcmp(childType, 'matlab.graphics.chart.primitive.Line')
                        markerType = get(children(i), 'Marker');
                        if strcmp(markerType, '.') || strcmp(markerType, 'o') 
                            delete(children(i));
                        end
                    end
                end
                
                % Plot samples
                plot(ax, xpos, ypos, 'r.', 'MarkerSize', 2, 'DisplayName', 'Samples');
                
                % Plot ground truth up to current step
                plot(ax, simState.X_gt(1, 1:(step+1)), simState.X_gt(2, 1:(step+1)), ...
                     'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
                
                trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
                modelName = 'Odometry';
                if simState.isDeadReckoning
                    modelName = 'Dead Reckoning';
                end
                title(ax, sprintf('%s Model - %s Trajectory - Step %d/%d', ...
                                 modelName, trajectoryNames{simState.trajectoryType}, step, size_U));
                
                drawnow;
                pause(paceValue);  % Use user-defined pace
            end

        end

        % Simulation finished
        simState.isRunning  = false;
        simState.isFinished = true;
        set(btnRunModel, 'Enable', 'on');
        set(btnStopModel, 'Enable', 'off');
        set(btnContinueModel, 'Enable', 'off');
        
        % Enable save button after completing live simulation
        set(btnSaveResults, 'Enable', 'on');
        
        % Update title
        trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
        modelName = 'Odometry';
        if simState.isDeadReckoning
            modelName = 'Dead Reckoning';
        end
        title(ax, sprintf('%s Model - %s Trajectory - Complete (%d samples)', ...
                         modelName, trajectoryNames{simState.trajectoryType}, simState.numSamples));
    end
    
    function stopSimulation(~, ~)
        % Stop the live simulation
        simState.isRunning = false;
        simState.isPaused = true;
        set(btnStopModel, 'Enable', 'off');
        set(btnContinueModel, 'Enable', 'on');
        set(btnResetModel, 'Enable', 'on');
        set(btnRunModel, 'Enable', 'off');
    end
    
    function continueSimulation(~, ~)
        % Continue from where it stopped
        simState.isRunning = true;
        simState.isPaused = false;
        set(btnStopModel, 'Enable', 'on');
        set(btnContinueModel, 'Enable', 'off');
        set(btnResetModel, 'Enable', 'off');
        set(btnRunModel, 'Enable', 'off');
        
        runLiveSimulation();
    end
    
    function resetSimulation(~, ~)
        % Reset simulation to initial state
        simState.isRunning = false;
        simState.isPaused = false;
        simState.isFinished = false;
        simState.currentStep = 0;
        
        % Reset sample array to initial positions
        if simState.isDeadReckoning
            size_U = size(simState.U_list, 2);
            simState.X_samples = repmat(simState.x0, 1, simState.numSamples, size_U+1);
        else
            size_U = size(simState.U_odom, 2);
            simState.X_samples = repmat(simState.x0, 1, simState.numSamples, size_U+1);
        end
        
        % Clear and reset plot
        cla(ax);
        hold(ax, 'on');
        
        % Redraw occupancy map if present
        if ~isempty(simState.occMap)
            show(simState.occMap, 'Parent', ax);
            hold(ax, 'on');
        end
        
        % Plot only initial ground truth point
        plot(ax, simState.X_gt(1,1), simState.X_gt(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
        
        trajectoryNames = {'Circular', 'Rectangle', 'Rectangle with Obstacles'};
        modelName = 'Odometry';
        if simState.isDeadReckoning
            modelName = 'Dead Reckoning';
        end
        title(ax, sprintf('%s Model - %s Trajectory - Reset', ...
                         modelName, trajectoryNames{simState.trajectoryType}));
        legend(ax, 'Location', 'best');
        grid(ax, 'on');
        axis(ax, 'equal');
        
        % Update button states
        set(btnRunModel, 'Enable', 'on');
        set(btnStopModel, 'Enable', 'off');
        set(btnContinueModel, 'Enable', 'off');
        set(btnResetModel, 'Enable', 'off');
    end
    
    function saveResults(~, ~)
        % Save current simulation results to files
        
        % Check if simulation has been run
        if isempty(simState.X_samples)
            errordlg('No simulation data to save. Please run a simulation first.');
            return;
        end
        
        % Generate timestamp for unique filename
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        
        % Get trajectory name
        trajectoryNames = {'Circular', 'Rectangle', 'RectangleObstacles'};
        trajectoryName = trajectoryNames{simState.trajectoryType};
        
        % Get model name
        if simState.isDeadReckoning
            modelName = 'DeadReckoning';
        else
            modelName = 'Odometry';
        end
        
        % Create base filename
        baseFilename = sprintf('RobotSim_%s_%s_%s', modelName, trajectoryName, timestamp);
        
        % Ask user for save location
        [filename, pathname] = uiputfile(...
            {'*.mat', 'MATLAB Data (*.mat)'; '*.txt', 'Text File (*.txt)'}, ...
            'Save Simulation Results', ...
            fullfile(pwd, baseFilename));
        
        if filename == 0
            % User cancelled
            return;
        end
        
        % Remove extension from filename
        [~, filenameNoExt, ~] = fileparts(filename);
        
        try
            %% 1. Save figure as PNG
            figureFilename = fullfile(pathname, [filenameNoExt, '.png']);
            
            % Temporarily hide control panel for cleaner figure export
            % Store original visibility state
            wasLiveSimChecked = get(chkLiveSimulation, 'Value');
            
            % Hide all control panel components
            set(modelPanelComponents, 'Visible', 'off');
            set(btnPanelModel, 'Visible', 'off');
            set(btnPanelSensors, 'Visible', 'off');
            set(btnPanelGNC, 'Visible', 'off');
            
            % Export figure
            exportgraphics(fig, figureFilename, 'Resolution', 300);
            
            % Restore control panel visibility
            set(modelPanelComponents, 'Visible', 'on');
            set(btnPanelModel, 'Visible', 'on');
            set(btnPanelSensors, 'Visible', 'on');
            set(btnPanelGNC, 'Visible', 'on');
            
            % Restore proper visibility based on state
            updateSliderVisibility();
            if wasLiveSimChecked
                set(btnStopModel, 'Visible', 'on');
                set(btnContinueModel, 'Visible', 'on');
                set(btnResetModel, 'Visible', 'on');
                set(txtPaceLabel, 'Visible', 'on');
                set(editPace, 'Visible', 'on');
            else
                set(btnStopModel, 'Visible', 'off');
                set(btnContinueModel, 'Visible', 'off');
                set(btnResetModel, 'Visible', 'off');
                set(txtPaceLabel, 'Visible', 'off');
                set(editPace, 'Visible', 'off');
            end
            
            %% 2. Save simulation data as MAT file
            matFilename = fullfile(pathname, [filenameNoExt, '.mat']);
            
            % Prepare data structure
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
            
            % Save MAT file
            save(matFilename, 'simulationData');
            
            %% 3. Save configuration info as TXT file
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
            
            %% Success message
            msgbox(sprintf('Results saved successfully!\n\nFiles created:\n• %s.png\n• %s.mat\n• %s_config.txt', ...
                   filenameNoExt, filenameNoExt, filenameNoExt), ...
                   'Save Successful', 'help');
            
        catch ME
            errordlg(sprintf('Error saving results: %s', ME.message), 'Save Error');
        end
    end

    %% Trajectory Generation Functions
    
    function [X_gt, U_list, x0, dt] = generateCircularTrajectory()
        % Generate circular trajectory
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
        % Generate rectangle trajectory
        x0 = [0; 0; 0];
        X_gt = x0;
        U_list = [];
        x = x0;
        
        % Go straight (4 steps)
        v = 2;
        w = 0;
        dt = 1;
        u = [v; w];
        for i = 1:4
            x = GTdeadReckoningMotionModel(x, u, dt);
            U_list = [U_list, u];
            X_gt = [X_gt, x];
        end
        
        % Turn 90 degrees
        v = 0;
        w = pi/2;
        dt = 1;
        u = [v; w];
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
        
        % Go straight again (4 steps)
        v = 2;
        w = 0;
        dt = 1;
        u = [v; w];
        for i = 1:4
            x = GTdeadReckoningMotionModel(x, u, dt);
            U_list = [U_list, u];
            X_gt = [X_gt, x];
        end
        
        % Turn 90 degrees again
        v = 0;
        w = pi/2;
        dt = 1;
        u = [v; w];
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
        
        % Go straight again (4 steps)
        v = 2;
        w = 0;
        dt = 1;
        u = [v; w];
        for i = 1:4
            x = GTdeadReckoningMotionModel(x, u, dt);
            U_list = [U_list, u];
            X_gt = [X_gt, x];
        end
        
        % Note: dt is returned as the last dt value (1.0)
    end
    
    function [X_gt, U_list, x0, dt, occMap] = generateRectangleObstacleTrajectory()
        % Generate rectangle trajectory with obstacles
        [X_gt, U_list, x0, dt] = generateRectangleTrajectory();
        
        % Create occupancy map
        occMap = createOccupancyMapWithObstacles();

    end

end

% filepath: c:\Users\user\Desktop\githubdeneme\prob_robot_project\createModelSection.m
function handles = createModelSection(parent, yOffset)
% CREATEMODELSECTION Creates the Model section UI components
%   handles = createModelSection(parent, yOffset)
%   Returns a struct with all UI component handles

    handles = struct();
    
    % Model Section Header
    handles.modelLabel = uilabel(parent, ...
        'Text', 'Model', ...
        'FontSize', 16, ...
        'FontWeight', 'bold', ...
        'Position', [20, yOffset, 200, 22]);
    
    % Robot Type
    handles.robotTypeLabel = uilabel(parent, ...
        'Text', 'Robot Type:', ...
        'Position', [20, yOffset-30, 100, 22]);
    handles.robotTypeDropdown = uidropdown(parent, ...
        'Items', {'Differential Drive', 'Ackermann', 'Omni-Directional'}, ...
        'Value', 'Differential Drive', ...
        'Position', [150, yOffset-30, 150, 22]);
    
    % Wheel Radius
    handles.wheelRadiusLabel = uilabel(parent, ...
        'Text', 'Wheel Radius (m):', ...
        'Position', [20, yOffset-60, 120, 22]);
    handles.wheelRadiusField = uieditfield(parent, 'numeric', ...
        'Value', 0.05, ...
        'Position', [150, yOffset-60, 100, 22], ...
        'Limits', [0.01, 1], ...
        'ValueDisplayFormat', '%.3f');
    
    % Wheel Base
    handles.wheelBaseLabel = uilabel(parent, ...
        'Text', 'Wheel Base (m):', ...
        'Position', [20, yOffset-90, 120, 22]);
    handles.wheelBaseField = uieditfield(parent, 'numeric', ...
        'Value', 0.3, ...
        'Position', [150, yOffset-90, 100, 22], ...
        'Limits', [0.1, 2], ...
        'ValueDisplayFormat', '%.3f');
    
    % Max Velocity
    handles.maxVelLabel = uilabel(parent, ...
        'Text', 'Max Velocity (m/s):', ...
        'Position', [20, yOffset-120, 120, 22]);
    handles.maxVelField = uieditfield(parent, 'numeric', ...
        'Value', 1.0, ...
        'Position', [150, yOffset-120, 100, 22], ...
        'Limits', [0.1, 10], ...
        'ValueDisplayFormat', '%.2f');
    
    % Max Angular Velocity
    handles.maxAngVelLabel = uilabel(parent, ...
        'Text', 'Max Ang Vel (rad/s):', ...
        'Position', [20, yOffset-150, 120, 22]);
    handles.maxAngVelField = uieditfield(parent, 'numeric', ...
        'Value', 2.0, ...
        'Position', [150, yOffset-150, 100, 22], ...
        'Limits', [0.1, 10], ...
        'ValueDisplayFormat', '%.2f');
    
    % Separator line
    handles.separator = uipanel(parent, ...
        'Position', [20, yOffset-170, 280, 2], ...
        'BorderType', 'line');
end
