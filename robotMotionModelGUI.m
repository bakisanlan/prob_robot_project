function robotMotionModelGUI()
    %ROBOTMOTIONMODELGUI Interactive GUI for robot motion model visualization
    close all; clc; clear
   
    % Add path of custom functions
    addpath('functions')
    addpath('gui_panels')
    
    % Initialize shared state
    sharedState = initializeSharedState();
    
    % Create main figure
    [fig, ax, dims] = createMainFigure();
    
    % Create panel selection buttons
    [btnPanelModel, btnPanelSensors, btnPanelGNC, separatorLine] = createPanelButtons(fig, dims);
    
    % Create all panels (initially hidden except MODEL)
    modelPanel = createModelPanel(fig, ax, dims, sharedState);
    sensorsPanel = createSensorsPanel(fig, ax, dims, sharedState);
    gncPanel = createGNCPanel(fig, ax, dims, sharedState);
    
    % Store panel data
    panels = struct();
    panels.MODEL = modelPanel;
    panels.SENSORS = sensorsPanel;
    panels.GNC = gncPanel;
    panels.current = 'MODEL';
    
    % Set panel button callbacks
    set(btnPanelModel, 'Callback', @(~,~)switchPanel('MODEL'));
    set(btnPanelSensors, 'Callback', @(~,~)switchPanel('SENSORS'));
    set(btnPanelGNC, 'Callback', @(~,~)switchPanel('GNC'));
    
    % Set resize callback
    set(fig, 'SizeChangedFcn', @resizeCallback);
    
    % Show initial panel
    switchPanel('MODEL');
    
    %% Callback Functions
    
    function switchPanel(panelName)
        % Hide all panels
        hidePanel(panels.MODEL);
        hidePanel(panels.SENSORS);
        hidePanel(panels.GNC);
        
        % Reset button colors
        set(btnPanelModel, 'BackgroundColor', [0.7, 0.7, 0.7]);
        set(btnPanelSensors, 'BackgroundColor', [0.7, 0.7, 0.7]);
        set(btnPanelGNC, 'BackgroundColor', [0.7, 0.7, 0.7]);
        
        % Show selected panel and highlight button
        panels.current = panelName;
        switch panelName
            case 'MODEL'
                showPanel(panels.MODEL);
                set(btnPanelModel, 'BackgroundColor', [0.3, 0.6, 0.9]);
            case 'SENSORS'
                showPanel(panels.SENSORS);
                set(btnPanelSensors, 'BackgroundColor', [0.3, 0.6, 0.9]);
            case 'GNC'
                showPanel(panels.GNC);
                set(btnPanelGNC, 'BackgroundColor', [0.3, 0.6, 0.9]);
        end
    end
    
    function resizeCallback(src, ~)
        if ~isvalid(src)
            return;
        end
        
        try
            % Get new figure size
            figPos = get(src, 'Position');
            newDims = calculateDimensions(figPos(3), figPos(4));
            
            % Update axes
            set(ax, 'Position', [newDims.plotX, newDims.plotY, ...
                                newDims.plotWidth, newDims.plotHeight]);
            
            % Update panel buttons and separator
            updatePanelButtons(btnPanelModel, btnPanelSensors, btnPanelGNC, newDims);
            
            % Update separator line
            if isvalid(separatorLine)
                btnY = newDims.figHeight * 0.929;
                set(separatorLine, 'Position', [newDims.panelX, btnY-5, newDims.panelWidth, 2]);
            end
            
            % Update active panel components
            switch panels.current
                case 'MODEL'
                    resizeModelPanel(panels.MODEL, newDims);
                case 'SENSORS'
                    resizeSensorsPanel(panels.SENSORS, newDims);
                case 'GNC'
                    resizeGNCPanel(panels.GNC, newDims);
            end
        catch ME
            % Silently catch errors during resize
        end
    end
end
