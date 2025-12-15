function [X_gt, U_list, x0, dt] = generateRectangleTrajectory()
%GENERATERECTANGLETRAJECTORY Generate rectangle trajectory for simulation
%
%   [X_gt, U_list, x0, dt] = generateRectangleTrajectory()
%
%   Outputs:
%       X_gt    - [3×N] Ground truth poses [x; y; theta]
%       U_list  - [2×(N-1)] Control inputs [v; w]
%       x0      - [3×1] Initial pose
%       dt      - Time step (seconds)

    % Initial pose
    x0 = [0; 0; 0];
    X_gt = x0;
    U_list = [];
    x = x0;
    
    % Segment 1: Go straight (4 steps)
    v = 2;
    w = 0;
    dt = 0.1;
    u = [v; w];
    for i = 1:40
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Turn 90 degrees
    v = 0;
    w = pi/2;
    dt = 0.1;
    u = [v; w];
    for i = 1:10
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Segment 2: Go straight again (4 steps)
    v = 2;
    w = 0;
    dt = 0.1;
    u = [v; w];
    for i = 1:40
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Turn 90 degrees again
    v = 0;
    w = pi/2;
    dt = 0.1;
    u = [v; w];
    for i = 1:10
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Segment 3: Go straight again (4 steps)
    v = 2;
    w = 0;
    dt = 0.1;
    u = [v; w];
    for i = 1:40
        x = GTdeadReckoningMotionModel(x, u, dt);
        U_list = [U_list, u];
        X_gt = [X_gt, x];
    end
    
    % Note: dt is returned as the last dt value (1.0)
end
