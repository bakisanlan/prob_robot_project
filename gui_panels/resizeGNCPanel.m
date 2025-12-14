function resizeGNCPanel(panel, dims)
    %RESIZEGNCPANEL Update GNC panel component positions
    
    if isfield(panel, 'txtTitle') && isvalid(panel.txtTitle)
        set(panel.txtTitle, 'Position', [dims.panelX, dims.figHeight*0.86, ...
                                         dims.panelWidth, dims.figHeight*0.04]);
    end
    
    if isfield(panel, 'txtPlaceholder') && isvalid(panel.txtPlaceholder)
        set(panel.txtPlaceholder, 'Position', [dims.panelX + dims.panelWidth*0.05, dims.figHeight*0.4, ...
                                               dims.panelWidth*0.9, dims.figHeight*0.15]);
    end
end
