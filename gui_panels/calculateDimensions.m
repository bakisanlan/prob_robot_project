function dims = calculateDimensions(figWidth, figHeight)
    %CALCULATEDIMENSIONS Calculate all GUI element dimensions
    
    dims = struct();
    dims.figWidth = figWidth;
    dims.figHeight = figHeight;
    
    dims.plotWidth = figWidth * 0.667;
    dims.plotHeight = figHeight * 0.857;
    dims.plotX = figWidth * 0.042;
    dims.plotY = figHeight * 0.071;
    
    dims.panelX = figWidth * 0.75;
    dims.panelWidth = figWidth * 0.233;
end
