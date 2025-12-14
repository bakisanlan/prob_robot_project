function panel = createGNCPanel(fig, ax, dims, sharedState)
    %CREATEGNCPANEL Create the GNC panel components
    
    panel = struct();
    
    % Title positioned below the panel buttons separator (buttons are at 0.929)
    % Start content area at 0.92 to be just below buttons
    txtTitle = uicontrol('Parent', fig, ...
              'Style', 'text', ...
              'String', 'GNC Panel', ...
              'Position', [dims.panelX, dims.figHeight*0.86, dims.panelWidth, dims.figHeight*0.04], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 12, 'FontWeight', 'bold', ...
              'Visible', 'off');
    
    % Placeholder in the middle of the panel area (between 0.1 and 0.86)
    txtPlaceholder = uicontrol('Parent', fig, ...
              'Style', 'text', ...
              'String', 'Guidance, Navigation, and Control options will be added here...', ...
              'Position', [dims.panelX + dims.panelWidth*0.05, dims.figHeight*0.4, dims.panelWidth*0.9, dims.figHeight*0.15], ...
              'HorizontalAlignment', 'center', ...
              'FontSize', 10, ...
              'Visible', 'off');
    
    panel.components = [txtTitle, txtPlaceholder];
    panel.txtTitle = txtTitle;
    panel.txtPlaceholder = txtPlaceholder;
end

function hidePanel(panel)
    set(panel.components, 'Visible', 'off');
end

function showPanel(panel)
    set(panel.components, 'Visible', 'on');
end

function resizeGNCPanel(panel, dims)
    % Update component positions based on new dimensions
end
