function [fig, ax, dims] = createMainFigure()
    %CREATEMAINFIGURE Create main figure and axes for the GUI
    
    % Get screen size
    screenSize = get(0, 'ScreenSize');
    screenWidth = screenSize(3);
    screenHeight = screenSize(4);
    
    % Define desired figure size
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
    
    % Create figure
    fig = figure('Name', 'Robot Motion Model Simulator', ...
                 'NumberTitle', 'off', ...
                 'Units', 'pixels', ...
                 'Position', [figX, figY, figWidth, figHeight], ...
                 'Resize', 'on', ...
                 'WindowState', 'maximized');
    
    % Calculate dimensions
    dims = calculateDimensions(figWidth, figHeight);
    
    % Create main axes
    ax = axes('Parent', fig, ...
              'Units', 'pixels', ...
              'Position', [dims.plotX, dims.plotY, dims.plotWidth, dims.plotHeight]);
    axis equal;
    grid on;
    hold(ax, 'on');
    title(ax, 'Robot Pose Samples');
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');
end

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
