function [X_gt, U_list, x0, dt] = generateCircularTrajectory()
%GENERATECIRCULARTRAJECTORY Generate circular trajectory for simulation
%
%   [X_gt, U_list, x0, dt] = generateCircularTrajectory()
%
%   Outputs:
%       X_gt    - [3×N] Ground truth poses [x; y; theta]
%       U_list  - [2×(N-1)] Control inputs [v; w]
%       x0      - [3×1] Initial pose
%       dt      - Time step (seconds)

    % Initial pose
    x0 = [0; 0; -pi/2];
    
    % Velocity parameters
    v = 1;      % linear velocity (m/s)
    r = 1;      % radius (m)
    w = -v/r;   % angular velocity (rad/s)
    
    % Time step for 8 segments (full circle)
    Tf = 2*pi/abs(w);
    dt = 0.1;
    %dt = abs((pi/4)/w);
    
    % Control input
    u = [v; w];
    
    % Generate trajectory
    x_gt = x0;
    X_gt = x_gt;
    U_list = [];
    
    for i = 1:round(Tf/dt)
        x_gt = GTdeadReckoningMotionModel(x_gt, u, dt);
        X_gt = [X_gt, x_gt];
        U_list = [U_list, u];
    end
end
