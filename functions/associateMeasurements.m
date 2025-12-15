function [z_actual, z_pred_matched, H_matched, R_matched] = associateMeasurements(detectedLandmarks, measurements, validLandmarks, z_pred, H, R)
%ASSOCIATEMEASUREMENTS Match actual landmark detections with predictions for EKF update
%
%   [z_actual, z_pred_matched, H_matched, R_matched] = associateMeasurements(detectedLandmarks, measurements, validLandmarks, z_pred, H, R)
%
%   Inputs:
%       detectedLandmarks : [K×3] landmarks detected by lidar [id, x, y]
%                           (from detectLandmarksFromScan)
%       measurements      : struct with .ranges and .bearings from actual scan
%       validLandmarks    : [M×3] landmarks predicted to be visible
%                           (from predictMeasurement)
%       z_pred            : [2M×1] predicted measurements
%       H                 : [2M×3] measurement Jacobian
%       R                 : [2×2] measurement noise covariance
%
%   Outputs:
%       z_actual       : [2N×1] actual measurements for matched landmarks
%       z_pred_matched : [2N×1] predicted measurements for matched landmarks
%       H_matched      : [2N×3] Jacobian for matched landmarks
%       R_matched      : [2N×2N] measurement covariance for matched landmarks
%
%   Data Association:
%       Matches landmarks by ID between detected and predicted sets.
%       Only landmarks present in BOTH sets are included in output.
%       This ensures z_actual and z_pred_matched have same dimensions.
%
%   Note: If no common landmarks, returns empty arrays.

    % Handle empty inputs
    if isempty(detectedLandmarks) || isempty(validLandmarks)
        z_actual = [];
        z_pred_matched = [];
        H_matched = [];
        R_matched = [];
        return;
    end
    
    % Extract landmark IDs
    detectedIDs = detectedLandmarks(:, 1);
    validIDs = validLandmarks(:, 1);
    
    % Find common landmarks (intersection)
    [commonIDs, idxDetected, idxValid] = intersect(detectedIDs, validIDs);
    
    N = length(commonIDs);
    
    if N == 0
        % No matching landmarks
        z_actual = [];
        z_pred_matched = [];
        H_matched = [];
        R_matched = [];
        return;
    end
    
    % Build matched measurement vectors
    z_actual = zeros(2*N, 1);
    z_pred_matched = zeros(2*N, 1);
    H_matched = zeros(2*N, 3);
    R_matched = zeros(2*N, 2*N);
    
    for i = 1:N
        % Actual measurements (from detected landmarks)
        z_actual(2*i-1) = measurements.ranges(idxDetected(i));
        z_actual(2*i)   = measurements.bearings(idxDetected(i));
        
        % Predicted measurements (from valid landmarks)
        z_pred_matched(2*i-1) = z_pred(2*idxValid(i)-1);
        z_pred_matched(2*i)   = z_pred(2*idxValid(i));
        
        % Jacobian rows
        H_matched(2*i-1:2*i, :) = H(2*idxValid(i)-1:2*idxValid(i), :);
        
        % Measurement covariance (block diagonal)
        R_matched(2*i-1:2*i, 2*i-1:2*i) = R;
    end
end

%% Sample usage (typical EKF workflow):
% % 1. Get actual lidar scan and detect landmarks
% [detectedLandmarks, measurements] = detectLandmarksFromScan(robotPose, ranges, angles, allLandmarks, 0.05);
% 
% % 2. Predict what should be visible from current state estimate
% sensorParams.maxRange = 10;
% sensorParams.fov = 90;
% [z_pred, H, validLandmarks] = predictMeasurement(x_pred, allLandmarks, sensorParams);
% 
% % 3. Associate actual detections with predictions
% R = diag([0.1^2, deg2rad(2)^2]);  % Measurement noise
% [z_actual, z_pred_matched, H_matched, R_matched] = associateMeasurements(...
%     detectedLandmarks, measurements, validLandmarks, z_pred, H, R);
% 
% % 4. Compute innovation for EKF update
% if ~isempty(z_actual)
%     innovation = z_actual - z_pred_matched;
%     innovation(2:2:end) = wrapToPi(innovation(2:2:end));  % Wrap bearing errors
%     
%     % Use innovation, H_matched, R_matched for EKF update
% end
