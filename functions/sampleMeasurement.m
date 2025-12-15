function z = sampleMeasurement(x_true, landmarks, R)
%SAMPLEMEASUREMENT Generate noisy range-bearing measurements to landmarks
%
%   z = sampleMeasurement(x_true, landmarks, R)
%
%   Inputs:
%       x_true    : [3×1] true robot pose [x; y; theta]
%       landmarks : [N×3] matrix of detected landmarks [id, x, y]
%                   Only landmarks detected by lidar should be included
%       R         : [2×2] measurement noise covariance matrix
%                   R = [sigma_range^2,        0;
%                        0,              sigma_bearing^2]
%                   Example: R = diag([0.1^2, deg2rad(2)^2])
%
%   Outputs:
%       z : [2N×1] noisy measurements [range1; bearing1; range2; bearing2; ...]
%
%   This function generates realistic sensor measurements by:
%   1. Computing ideal range-bearing to each landmark
%   2. Adding Gaussian noise based on covariance matrix R
%
%   Use this for simulating actual sensor readings in EKF testing.

    % Extract robot pose
    x = x_true(1);
    y = x_true(2);
    theta = x_true(3);
    
    % Number of landmarks
    N = size(landmarks, 1);
    
    % Initialize measurement vector
    z = zeros(2*N, 1);
    
    % Extract noise standard deviations
    sigma_range = sqrt(R(1,1));
    sigma_bearing = sqrt(R(2,2));
    
    % Process each landmark
    for i = 1:N
        % Landmark position
        lx = landmarks(i, 2);
        ly = landmarks(i, 3);
        
        % Relative position
        delta_x = lx - x;
        delta_y = ly - y;
        
        % Ideal measurements
        range_ideal = sqrt(delta_x^2 + delta_y^2);
        bearing_ideal = atan2(delta_y, delta_x) - theta;
        bearing_ideal = wrapToPi(bearing_ideal);
        
        % Add Gaussian noise
        range_noisy = range_ideal + sigma_range * randn;
        bearing_noisy = bearing_ideal + sigma_bearing * randn;
        bearing_noisy = wrapToPi(bearing_noisy);
        
        % Ensure range is positive
        range_noisy = max(range_noisy, 0.01);
        
        % Store in measurement vector
        z(2*i-1) = range_noisy;
        z(2*i)   = bearing_noisy;
    end
end

%% Sample usage:
% x_true = [2; 3; pi/4];                    % True robot pose
% landmarks = [1, 5, 7;                     % Landmark 1 at (5, 7)
%              2, 8, 4];                    % Landmark 2 at (8, 4)
% R = diag([0.1^2, deg2rad(2)^2]);         % Range: 0.1m std, Bearing: 2° std
% 
% z = sampleMeasurement(x_true, landmarks, R);
% 
% % z will contain noisy measurements [range1; bearing1; range2; bearing2]
