function [detectedLandmarks, measurements] = detectLandmarksFromScan(robotPose, ranges, angles, landmarks, landmarkRadius, rangeNoiseStd, bearingNoiseStd)
%DETECTLANDMARKSFROMSCAN Detect landmarks using raycasting with circular landmark model
%
%   [detectedLandmarks, measurements] = detectLandmarksFromScan(robotPose, ranges, angles, landmarks, landmarkRadius, rangeNoiseStd, bearingNoiseStd)
%
%   Inputs:
%       robotPose         : [3×1] robot pose [x; y; theta]
%       ranges            : [1×N] lidar range measurements (from actual scan)
%       angles            : [1×N] lidar angles relative to robot heading
%       landmarks         : [M×3] all landmarks [id, x, y]
%       landmarkRadius    : scalar, radius of landmark circles (default: 0.05 m)
%       rangeNoiseStd     : scalar, range measurement noise std deviation (default: 0.1 m)
%       bearingNoiseStd   : scalar, bearing measurement noise std deviation in radians (default: 0.01 rad)
%
%   Outputs:
%       detectedLandmarks : [K×3] detected landmarks [id, x, y]
%       measurements      : struct with fields:
%                           .ranges    [K×1] - measured ranges to landmarks (with noise)
%                           .bearings  [K×1] - measured bearings to landmarks in radians (with noise)
%                           .rayIdx    [K×1] - indices of rays that hit each landmark
%                           .hitPoints [K×2] - [x, y] intersection points on landmark circles
%
%   Algorithm:
%       1. For each lidar ray, compute ray endpoint in world frame
%       2. Check if ray intersects any landmark circle
%       3. If intersection exists and is within measured range, landmark is detected
%       4. Compute range and bearing from robot to landmark center
%       5. Add Gaussian noise to range and bearing measurements
%
%   Note: If multiple landmarks are hit by same ray, closest one is returned.
%         If landmark is hit by multiple rays, the ray with minimum range is used.

    if nargin < 5
        landmarkRadius = 0.05;  % Default 5cm radius
    end
    if nargin < 6
        rangeNoiseStd = 0.1;  % Default 0.1m range noise
    end
    if nargin < 7
        bearingNoiseStd = 0.01;  % Default 0.01 rad bearing noise
    end
    
    % Extract robot pose
    xr = robotPose(1);
    yr = robotPose(2);
    theta = robotPose(3);
    
    % Number of rays and landmarks
    numRays = length(ranges);
    numLandmarks = size(landmarks, 1);
    
    % Initialize detection results
    landmarkHits = false(numLandmarks, 1);
    hitRanges = inf(numLandmarks, 1);
    hitBearings = zeros(numLandmarks, 1);
    hitRayIdx = zeros(numLandmarks, 1);
    hitPoints = zeros(numLandmarks, 2);
    
    % Process each lidar ray
    for i = 1:numRays
        range = ranges(i);
        angle = angles(i);
        
        % Skip invalid measurements
        if isnan(range) || isinf(range) || range <= 0
            continue;
        end
        
        % Ray in world frame
        worldAngle = theta + angle;
        
        % Ray start point (robot position)
        rayStart = [xr; yr];
        
        % Ray direction (unit vector)
        rayDir = [cos(worldAngle); sin(worldAngle)];
        
        % Check intersection with each landmark
        for j = 1:numLandmarks
            landmarkPos = [landmarks(j, 2); landmarks(j, 3)];
            landmarkID = landmarks(j, 1);
            
            % Ray-circle intersection test
            % Ray: P = rayStart + t * rayDir
            % Circle: ||P - landmarkPos||^2 = r^2
            
            % Vector from ray start to landmark center
            L = landmarkPos - rayStart;
            
            % Project L onto ray direction
            tca = dot(L, rayDir);
            
            % If projection is negative, landmark is behind ray
            if tca < 0
                continue;
            end
            
            % Distance from landmark center to ray
            d2 = dot(L, L) - tca^2;
            r2 = landmarkRadius^2;
            
            % No intersection if distance > radius
            if d2 > r2
                continue;
            end
            
            % Distance from projection point to intersection
            thc = sqrt(r2 - d2);
            
            % Two intersection points (entry and exit)
            t0 = tca - thc;  % Entry point
            t1 = tca + thc;  % Exit point
            
            % Use entry point (closest intersection)
            tHit = t0;
            
            % Check if intersection is within measured range
            % (with small tolerance for numerical errors)
            tolerance = 0.1;  % 10cm tolerance
            if tHit > 0 && tHit <= (range + tolerance)
                % Intersection point on circle
                hitPoint = rayStart + tHit * rayDir;
                
                % Range and bearing to landmark CENTER (not intersection point)
                deltaX = landmarks(j, 2) - xr;
                deltaY = landmarks(j, 3) - yr;
                rangeTo = sqrt(deltaX^2 + deltaY^2);
                bearingTo = atan2(deltaY, deltaX) - theta;
                bearingTo = wrapToPi(bearingTo);
                
                % If this landmark already hit, keep the closest detection
                if ~landmarkHits(j) || rangeTo < hitRanges(j)
                    landmarkHits(j) = true;
                    hitRanges(j) = rangeTo;
                    hitBearings(j) = bearingTo;
                    hitRayIdx(j) = i;
                    hitPoints(j, :) = hitPoint';
                end
            end
        end
    end
    
    % Extract detected landmarks
    detectedIdx = find(landmarkHits);
    numDetected = length(detectedIdx);
    
    if numDetected == 0
        % No landmarks detected - preserve dimensions
        detectedLandmarks = zeros(0, 3);  % Empty 0×3 array
        measurements = struct('ranges', zeros(0,1), 'bearings', zeros(0,1), ...
                             'rayIdx', zeros(0,1), 'hitPoints', zeros(0,2));
    else
        % Build output
        detectedLandmarks = landmarks(detectedIdx, :);
        
        % Add Gaussian noise to measurements
        noisyRanges = hitRanges(detectedIdx) + rangeNoiseStd * randn(numDetected, 1);
        noisyBearings = hitBearings(detectedIdx) + bearingNoiseStd * randn(numDetected, 1);
        
        % Ensure ranges are positive
        noisyRanges = max(noisyRanges, 0.01);
        
        % Wrap bearings to [-pi, pi]
        noisyBearings = wrapToPi(noisyBearings);
        
        measurements.ranges = noisyRanges;
        measurements.bearings = noisyBearings;
        measurements.rayIdx = hitRayIdx(detectedIdx);
        measurements.hitPoints = hitPoints(detectedIdx, :);
    end
end

%% Sample usage:
% robotPose = [2; 3; 0];
% ranges = lidarScan.Ranges;  % From actual lidar scan
% angles = lidarScan.Angles;
% landmarks = [1, 5, 4;       % All known landmarks
%              2, 7, 6];
% landmarkRadius = 0.05;      % 5cm radius circles
% 
% [detected, meas] = detectLandmarksFromScan(robotPose, ranges, angles, landmarks, landmarkRadius);
% 
% % detected will be Kx3 matrix of detected landmarks
% % meas.ranges will be Kx1 measured ranges
% % meas.bearings will be Kx1 measured bearings (radians)
