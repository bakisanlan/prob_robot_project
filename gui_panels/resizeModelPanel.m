function resizeModelPanel(panel, dims)
    %RESIZEMODELPANEL Update MODEL panel component positions during resize
    
    % Model selection
    if isfield(panel, 'txtModelLabel') && isvalid(panel.txtModelLabel)
        set(panel.txtModelLabel, 'Position', [dims.panelX, dims.figHeight*0.857, dims.panelWidth*0.6, dims.figHeight*0.036]);
    end
    
    if isfield(panel, 'btnDeadReckoning') && isvalid(panel.btnDeadReckoning)
        set(panel.btnDeadReckoning, 'Position', [dims.panelX, dims.figHeight*0.814, dims.panelWidth*0.5, dims.figHeight*0.036]);
    end
    
    if isfield(panel, 'btnOdometry') && isvalid(panel.btnOdometry)
        set(panel.btnOdometry, 'Position', [dims.panelX, dims.figHeight*0.771, dims.panelWidth*0.5, dims.figHeight*0.036]);
    end
    
    % Trajectory
    if isfield(panel, 'txtTrajectoryLabel') && isvalid(panel.txtTrajectoryLabel)
        set(panel.txtTrajectoryLabel, 'Position', [dims.panelX, dims.figHeight*0.72, dims.panelWidth*0.6, dims.figHeight*0.036]);
    end
    
    if isfield(panel, 'popupTrajectory') && isvalid(panel.popupTrajectory)
        set(panel.popupTrajectory, 'Position', [dims.panelX, dims.figHeight*0.684, dims.panelWidth*0.7, dims.figHeight*0.036]);
    end
    
    % Samples
    if isfield(panel, 'txtSamplesLabel') && isvalid(panel.txtSamplesLabel)
        set(panel.txtSamplesLabel, 'Position', [dims.panelX, dims.figHeight*0.63, dims.panelWidth*0.6, dims.figHeight*0.036]);
    end
    
    if isfield(panel, 'editSamples') && isvalid(panel.editSamples)
        set(panel.editSamples, 'Position', [dims.panelX, dims.figHeight*0.594, dims.panelWidth*0.4, dims.figHeight*0.036]);
    end
    
    % Alpha parameters
    if isfield(panel, 'txtAlphaLabel') && isvalid(panel.txtAlphaLabel)
        set(panel.txtAlphaLabel, 'Position', [dims.panelX, dims.figHeight*0.53, dims.panelWidth*0.6, dims.figHeight*0.036]);
    end
    
    % Sliders
    sliderSpacing = dims.figHeight * 0.046;
    sliderHeight = dims.figHeight * 0.029;
    
    if isfield(panel, 'sliderLabels') && isfield(panel, 'sliders') && isfield(panel, 'sliderValues')
        for i = 1:6
            yPos = dims.figHeight*0.5 - (i-1)*sliderSpacing;
            
            sliderLabelField = sprintf('alpha%d', i);
            if isfield(panel.sliderLabels, sliderLabelField) && isvalid(panel.sliderLabels.(sliderLabelField))
                set(panel.sliderLabels.(sliderLabelField), 'Position', [dims.panelX, yPos, dims.panelWidth*0.12, sliderHeight]);
            end
            
            if isfield(panel.sliders, sliderLabelField) && isvalid(panel.sliders.(sliderLabelField))
                set(panel.sliders.(sliderLabelField), 'Position', [dims.panelX+dims.panelWidth*0.14, yPos, dims.panelWidth*0.64, sliderHeight]);
            end
            
            if isfield(panel.sliderValues, sliderLabelField) && isvalid(panel.sliderValues.(sliderLabelField))
                set(panel.sliderValues.(sliderLabelField), 'Position', [dims.panelX+dims.panelWidth*0.79, yPos, dims.panelWidth*0.18, sliderHeight]);
            end
        end
    end
    
    % Live simulation
    if isfield(panel, 'chkLiveSimulation') && isvalid(panel.chkLiveSimulation)
        set(panel.chkLiveSimulation, 'Position', [dims.panelX, dims.figHeight*0.15, dims.panelWidth*0.5, dims.figHeight*0.036]);
    end
    
    if isfield(panel, 'txtPaceLabel') && isvalid(panel.txtPaceLabel)
        set(panel.txtPaceLabel, 'Position', [dims.panelX, dims.figHeight*0.11, dims.panelWidth*0.3, dims.figHeight*0.03]);
    end
    
    if isfield(panel, 'editPace') && isvalid(panel.editPace)
        set(panel.editPace, 'Position', [dims.panelX+dims.panelWidth*0.32, dims.figHeight*0.11, dims.panelWidth*0.25, dims.figHeight*0.03]);
    end
    
    % Buttons
    if isfield(panel, 'btnRun') && isvalid(panel.btnRun)
        set(panel.btnRun, 'Position', [dims.panelX, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045]);
    end
    
    if isfield(panel, 'btnStop') && isvalid(panel.btnStop)
        set(panel.btnStop, 'Position', [dims.panelX+dims.panelWidth*0.25, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045]);
    end
    
    if isfield(panel, 'btnContinue') && isvalid(panel.btnContinue)
        set(panel.btnContinue, 'Position', [dims.panelX+dims.panelWidth*0.5, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045]);
    end
    
    if isfield(panel, 'btnReset') && isvalid(panel.btnReset)
        set(panel.btnReset, 'Position', [dims.panelX+dims.panelWidth*0.75, dims.figHeight*0.06, dims.panelWidth*0.22, dims.figHeight*0.045]);
    end
    
    if isfield(panel, 'btnSave') && isvalid(panel.btnSave)
        set(panel.btnSave, 'Position', [dims.panelX, dims.figHeight*0.01, dims.panelWidth*0.5, dims.figHeight*0.04]);
    end
end
