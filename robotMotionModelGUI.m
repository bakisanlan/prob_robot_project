function robotMotionModelGUI()
%ROBOTMOTIONMODELGUI Interactive GUI for robot motion model visualization
%
%   robotMotionModelGUI()
%
%   This is the main entry point for the Robot Motion Model Simulator.
%   The GUI visualizes probabilistic motion models (Dead Reckoning and Odometry)
%   with particle-based uncertainty propagation.
%
%   Structure:
%       - gui/panels/       : Panel creation functions
%       - gui/callbacks/    : Callback handler classes
%       - gui/utils/        : Utility functions (resize, etc.)
%       - trajectories/     : Trajectory generation functions
%       - functions/        : Motion model implementations
%
%   See also: createModelPanel, createSensorsPanel, createGNCPanel, ModelCallbacks

    close all; clc; clear
    
    %% Add paths to subfolders
    addpath("functions");
    addpath("gui/panels");
    addpath("gui/callbacks");
    addpath("gui/utils");
    addpath("trajectories");
    
    %% Calculate figure dimensions
    screenSize = get(0, 'ScreenSize');
    screenWidth = screenSize(3);
    screenHeight = screenSize(4);
    
    % Define desired figure size (1200x700) as reference
    figWidth = min(1200, screenWidth * 0.9);
    figHeight = min(700, screenHeight * 0.85);
    
    % Maintain aspect ratio
    aspectRatio = 1200 / 700;
    if figWidth / figHeight > aspectRatio
        figWidth = figHeight * aspectRatio;
    else
        figHeight = figWidth / aspectRatio;
    end
    
    % Center the figure on screen
    figX = (screenWidth - figWidth) / 2;
    figY = (screenHeight - figHeight) / 2;
    
    %% Create main figure
    fig = figure('Name', 'Robot Motion Model Simulator', ...
                 'NumberTitle', 'off', ...
                 'Units', 'pixels', ...
                 'Position', [figX, figY, figWidth, figHeight], ...
                 'Resize', 'on', ...
                 'WindowState', 'maximized', ...
                 'SizeChangedFcn', @(src,~) resizeGUI(src, figHeight, []));
    
    %% Calculate layout dimensions
    plotWidth = figWidth * 0.667;
    plotHeight = figHeight * 0.857;
    plotX = figWidth * 0.042;
    plotY = figHeight * 0.071;
    
    panelX = figWidth * 0.75;
    panelWidth = figWidth * 0.233;
    
    %% Create main axes
    ax = axes('Parent', fig, ...
              'Units', 'pixels', ...
              'Position', [plotX, plotY, plotWidth, plotHeight]);
    axis equal;
    grid on;
    hold(ax, 'on');
    title(ax, 'Robot Pose Samples');
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');
    
    %% Create panel selection buttons
    btnWidth = panelWidth / 3 - 5;
    btnHeight = figHeight * 0.05;
    btnY = figHeight * 0.929;
    
    btnPanelModel = uicontrol('Style', 'pushbutton', ...
                              'String', 'MODEL', ...
                              'Position', [panelX, btnY, btnWidth, btnHeight], ...
                              'FontSize', 10, ...
                              'FontWeight', 'bold', ...
                              'BackgroundColor', [0.3, 0.6, 0.9], ...
                              'Callback', @(~,~) switchPanel('MODEL'));
    
    btnPanelSensors = uicontrol('Style', 'pushbutton', ...
                                'String', 'Sensors&Landmarks', ...
                                'Position', [panelX+btnWidth+5, btnY, btnWidth, btnHeight], ...
                                'FontSize', 9, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.7, 0.7, 0.7], ...
                                'Callback', @(~,~) switchPanel('SENSORS'));
    
    btnPanelGNC = uicontrol('Style', 'pushbutton', ...
                            'String', 'GNC', ...
                            'Position', [panelX+2*btnWidth+10, btnY, btnWidth, btnHeight], ...
                            'FontSize', 10, ...
                            'FontWeight', 'bold', ...
                            'BackgroundColor', [0.7, 0.7, 0.7], ...
                            'Callback', @(~,~) switchPanel('GNC'));
    
    % Separator line
    uicontrol('Style', 'text', ...
              'String', '', ...
              'Position', [panelX, btnY-5, panelWidth, 2], ...
              'BackgroundColor', [0.3, 0.3, 0.3], ...
              'Tag', 'mainSeparator');
    
    %% Create panels using external functions
    [modelPanelComponents, modelHandles] = createModelPanel(fig, panelX, panelWidth, figHeight);
    [sensorsPanelComponents, sensorsHandles] = createSensorsPanel(fig, panelX, panelWidth, figHeight);
    [gncPanelComponents, gncHandles] = createGNCPanel(fig, panelX, panelWidth, figHeight);
    
    %% Setup callbacks using callback classes
    modelCallbacks = ModelCallbacks(ax, modelHandles, fig);
    modelCallbacks.setupCallbacks();
    
    sensorsCallbacks = SensorsCallbacks(ax, sensorsHandles, fig);
    sensorsCallbacks.setupCallbacks();
    
    gncCallbacks = GNCCallbacks(ax, gncHandles, fig);
    gncCallbacks.setupCallbacks();
    
    % Store callbacks in figure appdata for cross-panel access
    setappdata(fig, 'modelCallbacks', modelCallbacks);
    setappdata(fig, 'sensorsCallbacks', sensorsCallbacks);
    setappdata(fig, 'gncCallbacks', gncCallbacks);
    
    %% Initialize with MODEL panel visible
    currentPanel = 'MODEL';
    modelCallbacks.updateSliderVisibility();
    
    %% Panel switching function
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
                modelCallbacks.updateSliderVisibility();
            case 'SENSORS'
                set(sensorsPanelComponents, 'Visible', 'on');
                set(btnPanelSensors, 'BackgroundColor', [0.3, 0.6, 0.9]);
            case 'GNC'
                set(gncPanelComponents, 'Visible', 'on');
                set(btnPanelGNC, 'BackgroundColor', [0.3, 0.6, 0.9]);
        end
    end
end
