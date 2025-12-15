function [x_est, P_est] = EKF_localization(x_prev, P_prev, u, z_actual, detectedLandmarks, allLandmarks, params)
%EKF_LOCALIZATION Extended Kalman Filter for robot localization with landmarks
%
%   [x_est, P_est] = EKF_localization(x_prev, P_prev, u, z_actual, detectedLandmarks, allLandmarks, params)
%
%   Inputs:
%       x_prev            : [3×1] previous state estimate [x; y; theta]
%       P_prev            : [3×3] previous state covariance matrix
%       u                 : control input (format depends on motion model)
%                           Dead Reckoning: [2×1] [v; w]
%                           Odometry: [6×1] [x_bar; y_bar; theta_bar; x_bar'; y_bar'; theta_bar']
%       z_actual          : [2K×1] actual measurements [range1; bearing1; ...]
%                           (from associateMeasurements)
%       detectedLandmarks : [K×3] detected landmarks [id, x, y]
%       allLandmarks      : [M×3] all known landmarks [id, x, y]
%       params            : struct with fields:
%                           .motionModel  - 'deadreckoning' or 'odometry'
%                           .dt           - time step (for dead reckoning)
%                           .alpha        - motion noise parameters [6×1] or [4×1]
%                           .R            - measurement noise covariance [2×2]
%                           .sensorParams - struct with .maxRange, .fov
%
%   Outputs:
%       x_est : [3×1] updated state estimate
%       P_est : [3×3] updated state covariance
%
%   Algorithm:
%       1. Prediction step: propagate state and covariance using motion model
%       2. Measurement update: correct prediction using landmark observations
%
%   Note: If no measurements available, returns predicted state without update.

    %% ===== PREDICTION STEP =====
    
    % Propagate state using motion model
    if strcmp(params.motionModel, 'deadreckoning')
        % Dead reckoning motion model
        % x_pred = GTdeadReckoningMotionModel(x_prev, u, params.dt);
        x_pred = sampleDeadReckoningMotionModel(x_prev, u, params.alpha, params.dt);
        
        % Compute motion model Jacobian
        G = computeMotionJacobianDR(x_prev, u, params.dt);
        
        % Process noise covariance
        Q = computeProcessNoiseDR(u, params.alpha, params.dt);
        
    elseif strcmp(params.motionModel, 'odometry')
        % Odometry motion model
        % x_pred = GTodometryMotionModel(x_prev, u);
        x_pred = sampleOdometryMotionModel(x_prev, u, params.alpha);

        % Compute motion model Jacobian
        G = computeMotionJacobianOdom(x_prev, u);
        
        % Process noise covariance
        Q = computeProcessNoiseOdom(u, params.alpha);
        
    else
        error('Unknown motion model: %s', params.motionModel);
    end
    
    % Wrap angle
    x_pred(3) = wrapToPi(x_pred(3));
    
    % Propagate covariance
    P_pred = G * P_prev * G' + Q;
    
    %% ===== MEASUREMENT UPDATE STEP =====
    
    % Check if we have measurements
    if isempty(z_actual) || isempty(detectedLandmarks)
        % No measurements available, return predicted state
        x_est = x_pred;
        P_est = P_pred;
        return;
    end
    
    % Predict measurements for all landmarks (considering sensor limits)
    [z_pred, H, validLandmarks] = predictMeasurement(x_pred, allLandmarks, params.sensorParams);
    
    % Associate actual detections with predictions
    [z_actual_matched, z_pred_matched, H_matched, R_matched] = associateMeasurements(...
        detectedLandmarks, struct('ranges', z_actual(1:2:end), 'bearings', z_actual(2:2:end)), ...
        validLandmarks, z_pred, H, params.R);
    
    % Check if we have matched measurements
    if isempty(z_actual_matched)
        % No matching landmarks, return predicted state
        x_est = x_pred;
        P_est = P_pred;
        return;
    end
    
    % Innovation (measurement residual)
    innovation = z_actual_matched - z_pred_matched;
    
    % Wrap bearing errors to [-pi, pi]
    innovation(2:2:end) = wrapToPi(innovation(2:2:end));
    
    % Innovation covariance
    S = H_matched * P_pred * H_matched' + R_matched;
    
    % Kalman gain
    K = P_pred * H_matched' / S;
    
    % State update
    x_est = x_pred + K * innovation;
    
    % Wrap angle
    x_est(3) = wrapToPi(x_est(3));
    
    % Covariance update (Joseph form for numerical stability)
    I = eye(3);
    P_est = (I - K * H_matched) * P_pred * (I - K * H_matched)' + K * R_matched * K';
    
    % Ensure symmetry
    P_est = (P_est + P_est') / 2;
end

%% ===== HELPER FUNCTIONS =====

function G = computeMotionJacobianDR(x, u, dt)
    %COMPUTEMOTIONJACOBIANDR Jacobian of dead reckoning motion model
    
    theta = x(3);
    v = u(1);
    w = u(2);
    
    % Initialize Jacobian
    G = eye(3);
    
    % Handle small angular velocity
    eps_w = 1e-6;
    
    if abs(w) > eps_w
        % Circular arc motion
        r = v / w;
        G(1, 3) = -r * cos(theta) + r * cos(theta + w*dt);
        G(2, 3) = -r * sin(theta) + r * sin(theta + w*dt);
    else
        % Straight line motion
        G(1, 3) = -v * dt * sin(theta);
        G(2, 3) = v * dt * cos(theta);
    end
end

function Q = computeProcessNoiseDR(u, alpha, dt)
    %COMPUTEPROCESSNOISEDR Process noise covariance for dead reckoning
    
    v = u(1);
    w = u(2);
    
    % Velocity variances (simplified model)
    var_v = alpha(1)*abs(v) + alpha(2)*abs(w);
    var_w = alpha(3)*abs(v) + alpha(4)*abs(w);
    var_gamma = alpha(5)*abs(v) + alpha(6)*abs(w);
    
    % Process noise in control space
    M = diag([var_v, var_w, var_gamma]) * dt^2;
    
    % Map to state space (simplified)
    Q = M;
end

function G = computeMotionJacobianOdom(x, u)
    %COMPUTEMOTIONJACOBIANODOM Jacobian of odometry motion model
    
    theta = x(3);
    
    % Unpack odometry
    x_bar = u(1);
    y_bar = u(2);
    th_bar = u(3);
    x_bar_p = u(4);
    y_bar_p = u(5);
    th_bar_p = u(6);
    
    % Compute deltas
    delta_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar) - th_bar;
    delta_trans = sqrt((x_bar - x_bar_p)^2 + (y_bar - y_bar_p)^2);
    delta_rot1 = wrapToPi(delta_rot1);
    
    % Jacobian
    G = eye(3);
    G(1, 3) = -delta_trans * sin(theta + delta_rot1);
    G(2, 3) = delta_trans * cos(theta + delta_rot1);
end

function Q = computeProcessNoiseOdom(u, alpha)
    %COMPUTEPROCESSNOISEODOM Process noise covariance for odometry
    
    % Unpack odometry
    x_bar = u(1);
    y_bar = u(2);
    th_bar = u(3);
    x_bar_p = u(4);
    y_bar_p = u(5);
    th_bar_p = u(6);
    
    % Compute deltas
    delta_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar) - th_bar;
    delta_trans = sqrt((x_bar - x_bar_p)^2 + (y_bar - y_bar_p)^2);
    delta_rot2 = th_bar_p - th_bar - delta_rot1;
    
    delta_rot1 = wrapToPi(delta_rot1);
    delta_rot2 = wrapToPi(delta_rot2);
    
    % Variances
    var_rot1 = alpha(1)*abs(delta_rot1) + alpha(2)*abs(delta_trans);
    var_trans = alpha(3)*abs(delta_trans) + alpha(4)*abs(delta_rot1 + delta_rot2);
    var_rot2 = alpha(1)*abs(delta_rot2) + alpha(2)*abs(delta_trans);
    
    % Process noise (simplified)
    Q = diag([var_trans, var_trans, var_rot1 + var_rot2]);
end

%% Sample usage:
% % Initialize
% x_est = [0; 0; 0];           % Initial state
% P_est = eye(3) * 0.1;        % Initial covariance
% 
% % Parameters
% params.motionModel = 'deadReckoning';
% params.dt = 0.1;
% params.alpha = [0.001; 0.001; 0.001; 0.001; 0.0001; 0.0001];
% params.R = diag([0.1^2, deg2rad(2)^2]);
% params.sensorParams.maxRange = 10;
% params.sensorParams.fov = 90;
% 
% % At each time step:
% u = [v; w];  % Control input
% [detectedLandmarks, measurements] = detectLandmarksFromScan(x_true, ranges, angles, allLandmarks, 0.05);
% z_actual = [measurements.ranges; measurements.bearings];
% z_actual = z_actual(:);  % Interleave ranges and bearings
% 
% [x_est, P_est] = EKF_localization(x_est, P_est, u, z_actual, detectedLandmarks, allLandmarks, params);
