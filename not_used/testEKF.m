%% EKF Localization Example - Complete Workflow
% This script demonstrates the full EKF localization pipeline with:
% - Dead Reckoning motion model
% - Landmark-based measurements using raycasting
% - Data association and filtering
%
% Run this script to test the EKF before integrating into GUI

clear; clc; close all;

%% 1. Setup Parameters

% Motion model parameters
params.motionModel = 'deadreckoning';  % or 'odometry'
params.dt = 0.1;  % 100ms time step
params.alpha = [0.001; 0.001; 0.001; 0.001; 0.0001; 0.0001];  % Motion noise

% Sensor parameters
params.sensorParams.maxRange = 10;  % 10m max range
params.sensorParams.fov = 90;       % 90° field of view
params.R = diag([0.1^2, deg2rad(2)^2]);  % Range: 0.1m std, Bearing: 2° std
landmarkRadius = 0.05;  % 5cm landmark circles

% Known landmarks (ground truth map)
allLandmarks = [
    1, 5, 3;
    2, 8, 6;
    3, 2, 7;
    4, 10, 2;
];

%% 2. Generate Ground Truth Trajectory

% Initial state
x_true = [0; 0; 0];  % Start at origin

% Control sequence (circular motion)
T = 100;  % 100 time steps (10 seconds)
U = repmat([1.0; 0.3], 1, T);  % v=1 m/s, w=0.3 rad/s

% Generate ground truth trajectory
X_true = zeros(3, T+1);
X_true(:, 1) = x_true;

for t = 1:T
    X_true(:, t+1) = GTdeadReckoningMotionModel(X_true(:, t), U(:, t), params.dt);
end

%% 3. Initialize EKF

x0 = x_true + [5; 5; deg2rad(10)];  % Initial estimate with error
P0 = diag([0.5^2, 0.5^2, deg2rad(10)^2]);  % Initial uncertainty

% Create EKF object
ekf = EKFLocalizer(x0, P0, allLandmarks, params);

% Storage for results
X_est = zeros(3, T+1);
X_est(:, 1) = x0;
P_trace = zeros(1, T+1);
P_trace(1) = trace(P0);

%% 4. Run EKF Simulation

fprintf('Running EKF simulation...\n');

for t = 1:T
    % Control input
    u = U(:, t);
    
    % Simulate lidar scan at current true position
    % (In real system, this comes from actual sensor)
    ranges = simulateLidarScan(X_true(:, t+1), allLandmarks, params.sensorParams, landmarkRadius);
    angles = linspace(-deg2rad(params.sensorParams.fov/2), ...
                      deg2rad(params.sensorParams.fov/2), ...
                      length(ranges));
    
    % Detect landmarks from scan
    [detectedLandmarks, measurements] = detectLandmarksFromScan(...
        X_true(:, t+1), ranges, angles, allLandmarks, landmarkRadius);
    
    % Prepare measurement vector (interleaved ranges and bearings)
    if ~isempty(detectedLandmarks)
        z_actual = zeros(2*size(detectedLandmarks, 1), 1);
        z_actual(1:2:end) = measurements.ranges;
        z_actual(2:2:end) = measurements.bearings;
    else
        z_actual = [];
    end
    
    % EKF Prediction step
    ekf.predict(u);
    
    % EKF Update step (if landmarks detected)
    if ~isempty(z_actual)
        ekf.update(z_actual, detectedLandmarks);
    end
    
    % Get current estimate
    [x_est, P_est] = ekf.getState();
    
    % Store results
    X_est(:, t+1) = x_est;
    P_trace(t+1) = trace(P_est);
    
    % Progress
    if mod(t, 20) == 0
        fprintf('  Step %d/%d: Error = [%.3f, %.3f, %.1f°], Trace(P) = %.4f\n', ...
                t, T, ...
                X_true(1,t+1) - x_est(1), ...
                X_true(2,t+1) - x_est(2), ...
                rad2deg(wrapToPi(X_true(3,t+1) - x_est(3))), ...
                trace(P_est));
    end
end

fprintf('EKF simulation complete!\n\n');

%% 5. Visualize Results

figure('Position', [100, 100, 1200, 500]);

% Trajectory plot
subplot(1, 2, 1);
hold on; grid on; axis equal;
plot(X_true(1,:), X_true(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(X_est(1,:), X_est(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate');
plot(allLandmarks(:,2), allLandmarks(:,3), 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Landmarks');
plot(X_true(1,1), X_true(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
xlabel('X (m)');
ylabel('Y (m)');
title('EKF Localization: Trajectory');
legend('Location', 'best');

% Error plot
subplot(1, 2, 2);
time = (0:T) * params.dt;
errors = X_true - X_est;
errors(3,:) = wrapToPi(errors(3,:));

plot(time, errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(time, errors(2,:), 'g-', 'LineWidth', 1.5);
plot(time, rad2deg(errors(3,:)), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error');
legend('X error (m)', 'Y error (m)', '\theta error (°)');
title('EKF Localization: Estimation Errors');

% Statistics
fprintf('=== Final Statistics ===\n');
fprintf('Final position error: %.3f m\n', norm(errors(1:2, end)));
fprintf('Final heading error:  %.2f°\n', rad2deg(abs(errors(3, end))));
fprintf('Mean position error:  %.3f m\n', mean(vecnorm(errors(1:2, :))));
fprintf('Mean heading error:   %.2f°\n', rad2deg(mean(abs(errors(3, :)))));

%% Helper Function: Simulate Lidar Scan

function ranges = simulateLidarScan(robotPose, landmarks, sensorParams, landmarkRadius)
    %SIMULATELIDARSCAN Simulate lidar ranges with landmark detection
    
    % Generate angles
    halfFOV = deg2rad(sensorParams.fov / 2);
    resolution = deg2rad(2);  % 2° resolution
    angles = -halfFOV:resolution:halfFOV;
    
    % Initialize ranges at max range
    ranges = sensorParams.maxRange * ones(1, length(angles));
    
    % Extract robot pose
    x = robotPose(1);
    y = robotPose(2);
    theta = robotPose(3);
    
    % Check each ray for landmark intersection
    for i = 1:length(angles)
        angle = angles(i);
        worldAngle = theta + angle;
        
        rayStart = [x; y];
        rayDir = [cos(worldAngle); sin(worldAngle)];
        
        % Check intersection with each landmark
        for j = 1:size(landmarks, 1)
            landmarkPos = [landmarks(j, 2); landmarks(j, 3)];
            
            % Ray-circle intersection
            L = landmarkPos - rayStart;
            tca = dot(L, rayDir);
            
            if tca < 0, continue; end
            
            d2 = dot(L, L) - tca^2;
            r2 = landmarkRadius^2;
            
            if d2 > r2, continue; end
            
            thc = sqrt(r2 - d2);
            tHit = tca - thc;
            
            if tHit > 0 && tHit < ranges(i)
                % Add measurement noise
                ranges(i) = tHit + sqrt(0.1^2) * randn;
                ranges(i) = max(ranges(i), 0.01);  % Ensure positive
            end
        end
    end
end
