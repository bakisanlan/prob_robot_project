function resizeGUI(src, figHeight, panelButtons)
%RESIZEGUI Handle figure resize events
%
%   resizeGUI(src, figHeight, panelButtons)
%
%   Inputs:
%       src          - Source figure handle
%       figHeight    - Original figure height (for reference)
%       panelButtons - Struct with panel button handles

    % Check if figure still exists and is valid
    if ~isvalid(src)
        return;
    end
    
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
        btnSensors = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Sensors&Landmarks');
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
        separatorHandle = findobj(src, 'Tag', 'mainSeparator');
        if ~isempty(separatorHandle)
            set(separatorHandle, 'Position', [newPanelX, newBtnY-5, newPanelWidth, 2]);
        end
        
        % Update MODEL panel components
        updateModelPanelPositions(src, newPanelX, newPanelWidth, newHeight);
        
        % Update SENSORS panel components
        updateSensorsPanelPositions(src, newPanelX, newPanelWidth, newHeight);
        
        % Update GNC panel components
        updateGNCPanelPositions(src, newPanelX, newPanelWidth, newHeight);
        
    catch ME
        % Silently catch errors during resize to prevent disruption
        % disp(ME.message);
    end
end

function updateModelPanelPositions(src, panelX, panelWidth, figHeight)
%UPDATEMODELPANELPOSITIONS Update MODEL panel component positions

    txtModelLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Motion Model:');
    if ~isempty(txtModelLabel)
        set(txtModelLabel, 'Position', [panelX, figHeight*0.857, panelWidth*0.6, figHeight*0.036]);
    end
    
    btnDR = findobj(src, 'Type', 'uicontrol', 'Style', 'radiobutton', 'String', 'Dead Reckoning');
    if ~isempty(btnDR)
        set(btnDR, 'Position', [panelX, figHeight*0.814, panelWidth*0.5, figHeight*0.036]);
    end
    
    btnOdom = findobj(src, 'Type', 'uicontrol', 'Style', 'radiobutton', 'String', 'Odometry');
    if ~isempty(btnOdom)
        set(btnOdom, 'Position', [panelX, figHeight*0.771, panelWidth*0.5, figHeight*0.036]);
    end
    
    txtTrajLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Trajectory Type:');
    if ~isempty(txtTrajLabel)
        set(txtTrajLabel, 'Position', [panelX, figHeight*0.72, panelWidth*0.6, figHeight*0.036]);
    end
    
    popupTraj = findobj(src, 'Tag', 'popupTrajectory');
    if ~isempty(popupTraj)
        set(popupTraj, 'Position', [panelX, figHeight*0.684, panelWidth*0.7, figHeight*0.036]);
    end
    
    txtSamplesLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Number of Samples:');
    if ~isempty(txtSamplesLabel)
        set(txtSamplesLabel, 'Position', [panelX, figHeight*0.63, panelWidth*0.6, figHeight*0.036]);
    end
    
    editSamp = findobj(src, 'Tag', 'editSamples');
    if ~isempty(editSamp)
        set(editSamp, 'Position', [panelX, figHeight*0.594, panelWidth*0.4, figHeight*0.036]);
    end
    
    txtAlphaLabel = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Alpha Parameters:');
    if ~isempty(txtAlphaLabel)
        set(txtAlphaLabel, 'Position', [panelX, figHeight*0.53, panelWidth*0.6, figHeight*0.036]);
    end
    
    % Update sliders using Tags
    sliderSpacing = figHeight * 0.046;
    sliderHeight = figHeight * 0.029;
    
    for i = 1:6
        yPos = figHeight*0.5 - (i-1)*sliderSpacing;
        
        sliderLabel = findobj(src, 'Tag', sprintf('sliderLabel%d', i));
        slider = findobj(src, 'Tag', sprintf('slider%d', i));
        sliderValue = findobj(src, 'Tag', sprintf('sliderValue%d', i));
        
        if ~isempty(sliderLabel)
            set(sliderLabel, 'Position', [panelX, yPos, panelWidth*0.12, sliderHeight]);
        end
        
        if ~isempty(slider)
            set(slider, 'Position', [panelX+panelWidth*0.14, yPos, panelWidth*0.64, sliderHeight]);
        end
        
        if ~isempty(sliderValue)
            set(sliderValue, 'Position', [panelX+panelWidth*0.79, yPos, panelWidth*0.18, sliderHeight]);
        end
    end
    
    % Update live simulation controls
    chkLive = findobj(src, 'Type', 'uicontrol', 'Style', 'checkbox', 'String', 'Live Simulation');
    if ~isempty(chkLive)
        set(chkLive, 'Position', [panelX, figHeight*0.15, panelWidth*0.5, figHeight*0.036]);
    end
    
    txtPace = findobj(src, 'Type', 'uicontrol', 'Style', 'text', 'String', 'Pace (s):');
    if ~isempty(txtPace)
        set(txtPace, 'Position', [panelX, figHeight*0.11, panelWidth*0.3, figHeight*0.03]);
    end
    
    % Update pace edit field
    editPaceField = findobj(src, 'Tag', 'editPace');
    if ~isempty(editPaceField)
        set(editPaceField, 'Position', [panelX+panelWidth*0.32, figHeight*0.11, panelWidth*0.25, figHeight*0.03]);
    end
    
    % Update simulation control buttons
    btnRun = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Run');
    if ~isempty(btnRun)
        set(btnRun, 'Position', [panelX, figHeight*0.06, panelWidth*0.22, figHeight*0.045]);
    end
    
    btnStop = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Stop');
    if ~isempty(btnStop)
        set(btnStop, 'Position', [panelX+panelWidth*0.25, figHeight*0.06, panelWidth*0.22, figHeight*0.045]);
    end
    
    btnCont = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Continue');
    if ~isempty(btnCont)
        set(btnCont, 'Position', [panelX+panelWidth*0.5, figHeight*0.06, panelWidth*0.22, figHeight*0.045]);
    end
    
    btnReset = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Reset');
    if ~isempty(btnReset)
        set(btnReset, 'Position', [panelX+panelWidth*0.75, figHeight*0.06, panelWidth*0.22, figHeight*0.045]);
    end
    
    btnSave = findobj(src, 'Type', 'uicontrol', 'Style', 'pushbutton', 'String', 'Save Results');
    if ~isempty(btnSave)
        set(btnSave, 'Position', [panelX, figHeight*0.01, panelWidth*0.5, figHeight*0.04]);
    end
end

function updateSensorsPanelPositions(src, panelX, panelWidth, figHeight)
%UPDATESENSORSPANELPOSITIONS Update SENSORS & LANDMARKS panel component positions

    % Title
    txtSensorsTitle = findobj(src, 'Tag', 'sensorsPanelTitle');
    if ~isempty(txtSensorsTitle)
        set(txtSensorsTitle, 'Position', [panelX, figHeight*0.857, panelWidth*0.95, figHeight*0.043]);
    end
    
    % Lidar Parameters label
    txtLidarParams = findobj(src, 'Tag', 'lidarParamsLabel');
    if ~isempty(txtLidarParams)
        set(txtLidarParams, 'Position', [panelX, figHeight*0.81, panelWidth*0.6, figHeight*0.027]);
    end
    
    % Range controls
    txtRangeLabel = findobj(src, 'Tag', 'lidarRangeLabel');
    if ~isempty(txtRangeLabel)
        set(txtRangeLabel, 'Position', [panelX, figHeight*0.775, panelWidth*0.45, figHeight*0.025]);
    end
    editRange = findobj(src, 'Tag', 'lidarRange');
    if ~isempty(editRange)
        set(editRange, 'Position', [panelX+panelWidth*0.5, figHeight*0.775, panelWidth*0.35, figHeight*0.025]);
    end
    
    % FOV controls
    txtFOVLabel = findobj(src, 'Tag', 'lidarFOVLabel');
    if ~isempty(txtFOVLabel)
        set(txtFOVLabel, 'Position', [panelX, figHeight*0.74, panelWidth*0.45, figHeight*0.025]);
    end
    editFOV = findobj(src, 'Tag', 'lidarFOV');
    if ~isempty(editFOV)
        set(editFOV, 'Position', [panelX+panelWidth*0.5, figHeight*0.74, panelWidth*0.35, figHeight*0.025]);
    end
    
    % Resolution controls
    txtResLabel = findobj(src, 'Tag', 'lidarResolutionLabel');
    if ~isempty(txtResLabel)
        set(txtResLabel, 'Position', [panelX, figHeight*0.705, panelWidth*0.45, figHeight*0.025]);
    end
    editRes = findobj(src, 'Tag', 'lidarResolution');
    if ~isempty(editRes)
        set(editRes, 'Position', [panelX+panelWidth*0.5, figHeight*0.705, panelWidth*0.35, figHeight*0.025]);
    end
    
    % Info section
    txtInfoLabel = findobj(src, 'Tag', 'lidarScanInfoLabel');
    if ~isempty(txtInfoLabel)
        set(txtInfoLabel, 'Position', [panelX, figHeight*0.665, panelWidth*0.4, figHeight*0.027]);
    end
    txtNumRays = findobj(src, 'Tag', 'numRaysDisplay');
    if ~isempty(txtNumRays)
        set(txtNumRays, 'Position', [panelX, figHeight*0.635, panelWidth*0.5, figHeight*0.023]);
    end
    txtAngleRange = findobj(src, 'Tag', 'angleRangeDisplay');
    if ~isempty(txtAngleRange)
        set(txtAngleRange, 'Position', [panelX, figHeight*0.61, panelWidth*0.8, figHeight*0.023]);
    end
    
    % Noise section
    txtNoiseLabel = findobj(src, 'Tag', 'lidarNoiseLabel');
    if ~isempty(txtNoiseLabel)
        set(txtNoiseLabel, 'Position', [panelX, figHeight*0.58, panelWidth*0.6, figHeight*0.023]);
    end
    txtRangeNoiseLabel = findobj(src, 'Tag', 'lidarRangeNoiseLabel');
    if ~isempty(txtRangeNoiseLabel)
        set(txtRangeNoiseLabel, 'Position', [panelX, figHeight*0.555, panelWidth*0.45, figHeight*0.020]);
    end
    editRangeNoise = findobj(src, 'Tag', 'lidarRangeNoise');
    if ~isempty(editRangeNoise)
        set(editRangeNoise, 'Position', [panelX+panelWidth*0.5, figHeight*0.555, panelWidth*0.35, figHeight*0.020]);
    end
    txtAngularNoiseLabel = findobj(src, 'Tag', 'lidarAngularNoiseLabel');
    if ~isempty(txtAngularNoiseLabel)
        set(txtAngularNoiseLabel, 'Position', [panelX, figHeight*0.530, panelWidth*0.45, figHeight*0.020]);
    end
    editAngularNoise = findobj(src, 'Tag', 'lidarAngularNoise');
    if ~isempty(editAngularNoise)
        set(editAngularNoise, 'Position', [panelX+panelWidth*0.5, figHeight*0.530, panelWidth*0.35, figHeight*0.020]);
    end
    
    % Checkboxes
    chkShowLidar = findobj(src, 'Tag', 'showLidarRays');
    if ~isempty(chkShowLidar)
        set(chkShowLidar, 'Position', [panelX, figHeight*0.50, panelWidth*0.6, figHeight*0.025]);
    end
    chkShowNoisy = findobj(src, 'Tag', 'showNoisyRays');
    if ~isempty(chkShowNoisy)
        set(chkShowNoisy, 'Position', [panelX, figHeight*0.47, panelWidth*0.6, figHeight*0.025]);
    end
    
    % Separator line
    separatorLidar = findobj(src, 'Tag', 'lidarSeparator');
    if ~isempty(separatorLidar)
        set(separatorLidar, 'Position', [panelX, figHeight*0.425, panelWidth, 2]);
    end
    
    % Lidar Buttons
    btnCreateScan = findobj(src, 'Tag', 'btnCreateScan');
    if ~isempty(btnCreateScan)
        set(btnCreateScan, 'Position', [panelX, figHeight*0.44, panelWidth*0.45, figHeight*0.025]);
    end
    btnClearScan = findobj(src, 'Tag', 'btnClearScan');
    if ~isempty(btnClearScan)
        set(btnClearScan, 'Position', [panelX+panelWidth*0.5, figHeight*0.44, panelWidth*0.45, figHeight*0.025]);
    end
    
    % Landmarks Section
    txtLandmarksTitle = findobj(src, 'Tag', 'landmarksTitle');
    if ~isempty(txtLandmarksTitle)
        set(txtLandmarksTitle, 'Position', [panelX, figHeight*0.38, panelWidth*0.6, figHeight*0.036]);
    end
    
    % Landmark listbox
    listLandmarks = findobj(src, 'Tag', 'listLandmarks');
    if ~isempty(listLandmarks)
        set(listLandmarks, 'Position', [panelX, figHeight*0.24, panelWidth*0.95, figHeight*0.15]);
    end
    
    % Manual Entry Section
    txtManualEntry = findobj(src, 'Tag', 'landmarkManualEntryLabel');
    if ~isempty(txtManualEntry)
        set(txtManualEntry, 'Position', [panelX, figHeight*0.20, panelWidth*0.5, figHeight*0.025]);
    end
    
    txtXLabel = findobj(src, 'Tag', 'landmarkXLabel');
    if ~isempty(txtXLabel)
        set(txtXLabel, 'Position', [panelX, figHeight*0.165, panelWidth*0.1, figHeight*0.025]);
    end
    
    editLandmarkX = findobj(src, 'Tag', 'editLandmarkX');
    if ~isempty(editLandmarkX)
        set(editLandmarkX, 'Position', [panelX+panelWidth*0.12, figHeight*0.165, panelWidth*0.35, figHeight*0.025]);
    end
    
    txtYLabel = findobj(src, 'Tag', 'landmarkYLabel');
    if ~isempty(txtYLabel)
        set(txtYLabel, 'Position', [panelX+panelWidth*0.5, figHeight*0.165, panelWidth*0.1, figHeight*0.025]);
    end
    
    editLandmarkY = findobj(src, 'Tag', 'editLandmarkY');
    if ~isempty(editLandmarkY)
        set(editLandmarkY, 'Position', [panelX+panelWidth*0.62, figHeight*0.165, panelWidth*0.35, figHeight*0.025]);
    end
    
    % Landmark Control Buttons
    btnAddManual = findobj(src, 'Tag', 'btnAddManual');
    if ~isempty(btnAddManual)
        set(btnAddManual, 'Position', [panelX, figHeight*0.125, panelWidth*0.45, figHeight*0.035]);
    end
    
    btnAddByClick = findobj(src, 'Tag', 'btnAddByClick');
    if ~isempty(btnAddByClick)
        set(btnAddByClick, 'Position', [panelX+panelWidth*0.5, figHeight*0.125, panelWidth*0.45, figHeight*0.035]);
    end
    
    btnRemoveLandmark = findobj(src, 'Tag', 'btnRemoveLandmark');
    if ~isempty(btnRemoveLandmark)
        set(btnRemoveLandmark, 'Position', [panelX, figHeight*0.085, panelWidth*0.45, figHeight*0.035]);
    end
    
    btnClearLandmarks = findobj(src, 'Tag', 'btnClearLandmarks');
    if ~isempty(btnClearLandmarks)
        set(btnClearLandmarks, 'Position', [panelX+panelWidth*0.5, figHeight*0.085, panelWidth*0.45, figHeight*0.035]);
    end
    
    % Status
    txtStatus = findobj(src, 'Tag', 'sensorStatus');
    if ~isempty(txtStatus)
        set(txtStatus, 'Position', [panelX, figHeight*0.04, panelWidth*0.95, figHeight*0.035]);
    end
end

function updateGNCPanelPositions(src, panelX, panelWidth, figHeight)
%UPDATEGNCPANELPOSITIONS Update GNC panel component positions

    % Title
    txtTitle = findobj(src, 'Tag', 'gncPanelTitle');
    if ~isempty(txtTitle)
        set(txtTitle, 'Position', [panelX, figHeight*0.857, panelWidth*0.95, figHeight*0.043]);
    end
    
    % Estimator selection
    txtEstimator = findobj(src, 'Tag', 'estimatorLabel');
    if ~isempty(txtEstimator)
        set(txtEstimator, 'Position', [panelX, figHeight*0.81, panelWidth*0.6, figHeight*0.027]);
    end
    popupEst = findobj(src, 'Tag', 'estimatorPopup');
    if ~isempty(popupEst)
        set(popupEst, 'Position', [panelX, figHeight*0.77, panelWidth*0.9, figHeight*0.036]);
    end
    
    % Initial Covariance
    txtInitCov = findobj(src, 'Tag', 'initCovLabel');
    if ~isempty(txtInitCov)
        set(txtInitCov, 'Position', [panelX, figHeight*0.72, panelWidth*0.7, figHeight*0.027]);
    end
    
    txtP0x = findobj(src, 'Tag', 'p0xLabel');
    if ~isempty(txtP0x)
        set(txtP0x, 'Position', [panelX, figHeight*0.685, panelWidth*0.25, figHeight*0.023]);
    end
    editP0x = findobj(src, 'Tag', 'editP0x');
    if ~isempty(editP0x)
        set(editP0x, 'Position', [panelX+panelWidth*0.28, figHeight*0.685, panelWidth*0.28, figHeight*0.023]);
    end
    
    txtP0y = findobj(src, 'Tag', 'p0yLabel');
    if ~isempty(txtP0y)
        set(txtP0y, 'Position', [panelX, figHeight*0.655, panelWidth*0.25, figHeight*0.023]);
    end
    editP0y = findobj(src, 'Tag', 'editP0y');
    if ~isempty(editP0y)
        set(editP0y, 'Position', [panelX+panelWidth*0.28, figHeight*0.655, panelWidth*0.28, figHeight*0.023]);
    end
    
    txtP0theta = findobj(src, 'Tag', 'p0thetaLabel');
    if ~isempty(txtP0theta)
        set(txtP0theta, 'Position', [panelX, figHeight*0.625, panelWidth*0.25, figHeight*0.023]);
    end
    editP0theta = findobj(src, 'Tag', 'editP0theta');
    if ~isempty(editP0theta)
        set(editP0theta, 'Position', [panelX+panelWidth*0.28, figHeight*0.625, panelWidth*0.28, figHeight*0.023]);
    end
    
    % Sensor Parameters (measurement noise from SENSORS panel)
    txtSensorParams = findobj(src, 'Tag', 'sensorParamsLabel');
    if ~isempty(txtSensorParams)
        set(txtSensorParams, 'Position', [panelX, figHeight*0.58, panelWidth*0.8, figHeight*0.027]);
    end
    txtSensorInfo = findobj(src, 'Tag', 'sensorInfo');
    if ~isempty(txtSensorInfo)
        set(txtSensorInfo, 'Position', [panelX, figHeight*0.545, panelWidth*0.9, figHeight*0.023]);
    end
    
    txtLandmarkRadius = findobj(src, 'Tag', 'landmarkRadiusLabel');
    if ~isempty(txtLandmarkRadius)
        set(txtLandmarkRadius, 'Position', [panelX, figHeight*0.51, panelWidth*0.5, figHeight*0.023]);
    end
    editLandmarkRadius = findobj(src, 'Tag', 'editLandmarkRadius');
    if ~isempty(editLandmarkRadius)
        set(editLandmarkRadius, 'Position', [panelX+panelWidth*0.55, figHeight*0.51, panelWidth*0.28, figHeight*0.023]);
    end
    
    % Separators
    sep1 = findobj(src, 'Tag', 'gncSeparator1');
    if ~isempty(sep1)
        set(sep1, 'Position', [panelX, figHeight*0.48, panelWidth, 2]);
    end
    
    % Visualization
    txtVis = findobj(src, 'Tag', 'visLabel');
    if ~isempty(txtVis)
        set(txtVis, 'Position', [panelX, figHeight*0.34, panelWidth*0.6, figHeight*0.027]);
    end
    
    chkEstimate = findobj(src, 'Tag', 'chkShowEstimate');
    if ~isempty(chkEstimate)
        set(chkEstimate, 'Position', [panelX, figHeight*0.31, panelWidth*0.7, figHeight*0.025]);
    end
    chkEllipsoid = findobj(src, 'Tag', 'chkShowEllipsoid');
    if ~isempty(chkEllipsoid)
        set(chkEllipsoid, 'Position', [panelX, figHeight*0.28, panelWidth*0.8, figHeight*0.025]);
    end
    chkDetections = findobj(src, 'Tag', 'chkShowDetections');
    if ~isempty(chkDetections)
        set(chkDetections, 'Position', [panelX, figHeight*0.25, panelWidth*0.8, figHeight*0.025]);
    end
    
    txtConfidence = findobj(src, 'Tag', 'confidenceLabel');
    if ~isempty(txtConfidence)
        set(txtConfidence, 'Position', [panelX, figHeight*0.215, panelWidth*0.4, figHeight*0.023]);
    end
    editConfidence = findobj(src, 'Tag', 'editConfidence');
    if ~isempty(editConfidence)
        set(editConfidence, 'Position', [panelX+panelWidth*0.45, figHeight*0.215, panelWidth*0.2, figHeight*0.023]);
    end
    txtConfInfo = findobj(src, 'Tag', 'confidenceInfo');
    if ~isempty(txtConfInfo)
        set(txtConfInfo, 'Position', [panelX, figHeight*0.19, panelWidth*0.9, figHeight*0.020]);
    end
    
    sep2 = findobj(src, 'Tag', 'gncSeparator2');
    if ~isempty(sep2)
        set(sep2, 'Position', [panelX, figHeight*0.165, panelWidth, 2]);
    end
    
    % Control Buttons
    btnRunEKF = findobj(src, 'Tag', 'btnRunEKF');
    if ~isempty(btnRunEKF)
        set(btnRunEKF, 'Position', [panelX, figHeight*0.12, panelWidth*0.45, figHeight*0.04]);
    end
    btnResetEKF = findobj(src, 'Tag', 'btnResetEKF');
    if ~isempty(btnResetEKF)
        set(btnResetEKF, 'Position', [panelX+panelWidth*0.5, figHeight*0.12, panelWidth*0.45, figHeight*0.04]);
    end
    
    chkLiveEKF = findobj(src, 'Tag', 'chkLiveEKF');
    if ~isempty(chkLiveEKF)
        set(chkLiveEKF, 'Position', [panelX, figHeight*0.08, panelWidth*0.5, figHeight*0.025]);
    end
    
    txtPaceEKF = findobj(src, 'Tag', 'ekfPaceLabel');
    if ~isempty(txtPaceEKF)
        set(txtPaceEKF, 'Position', [panelX, figHeight*0.05, panelWidth*0.3, figHeight*0.023]);
    end
    editPaceEKF = findobj(src, 'Tag', 'editEKFPace');
    if ~isempty(editPaceEKF)
        set(editPaceEKF, 'Position', [panelX+panelWidth*0.32, figHeight*0.05, panelWidth*0.25, figHeight*0.023]);
    end
    
    % Status
    txtStatus = findobj(src, 'Tag', 'gncStatus');
    if ~isempty(txtStatus)
        set(txtStatus, 'Position', [panelX, figHeight*0.01, panelWidth*0.95, figHeight*0.035]);
    end
end
