function updatePanelButtons(btnModel, btnSensors, btnGNC, dims)
    %UPDATEPANELBUTTONS Update panel button positions during resize
    
    btnWidth = dims.panelWidth / 3 - 5;
    btnHeight = dims.figHeight * 0.05;
    btnY = dims.figHeight * 0.929;
    
    if isvalid(btnModel)
        set(btnModel, 'Position', [dims.panelX, btnY, btnWidth, btnHeight]);
    end
    
    if isvalid(btnSensors)
        set(btnSensors, 'Position', [dims.panelX+btnWidth+5, btnY, btnWidth, btnHeight]);
    end
    
    if isvalid(btnGNC)
        set(btnGNC, 'Position', [dims.panelX+2*btnWidth+10, btnY, btnWidth, btnHeight]);
    end
end
