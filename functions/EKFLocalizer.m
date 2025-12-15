classdef EKFLocalizer < handle
%EKFLOCALIZER Extended Kalman Filter class for robot localization with landmarks
%
%   This class implements an Extended Kalman Filter for robot localization
%   using range-bearing measurements to known landmarks.
%
%   Properties:
%       mu          - [3×1] Current state estimate [x; y; theta]
%       Sigma       - [3×3] Current covariance matrix
%       params      - Struct with filter parameters
%       landmarks   - [N×3] Known landmarks [id, x, y]
%
%   Methods:
%       EKFLocalizer(x0, P0, landmarks, params) - Constructor
%       predict(u)                               - Prediction step
%       update(z, detectedLandmarks)            - Measurement update step
%       getState()                              - Get current state estimate
%       getCovariance()                         - Get current covariance
%
%   Example:
%       % Initialize EKF
%       x0 = [0; 0; 0];
%       P0 = diag([0.1, 0.1, 0.05]);
%       landmarks = [1, 5, 3; 2, 10, 7];
%       params.motionModel = 'deadreckoning';
%       params.dt = 0.1;
%       params.alpha = [0.001; 0.001; 0.001; 0.001; 0.0001; 0.0001];
%       params.R = diag([0.05, 0.01]);
%       params.sensorParams.maxRange = 10;
%       params.sensorParams.fov = pi/2;
%       
%       ekf = EKFLocalizer(x0, P0, landmarks, params);
%       
%       % Prediction step
%       u = [1.0; 0.1];  % [v; w] for dead reckoning
%       ekf.predict(u);
%       
%       % Update step (if landmarks detected)
%       z = [3.2; 0.5; 4.1; -0.3];  % [range1; bearing1; range2; bearing2]
%       detectedLandmarks = [1, 5, 3; 2, 10, 7];
%       ekf.update(z, detectedLandmarks);
%       
%       % Get current estimate
%       [x_est, P_est] = ekf.getState();

    properties
        mu          % [3×1] State estimate [x; y; theta]
        Sigma       % [3×3] Covariance matrix
        params      % Filter parameters struct
        landmarks   % [N×3] Known landmarks [id, x, y]
    end
    
    methods
        function obj = EKFLocalizer(x0, P0, landmarks, params)
            %EKFLOCALIZER Constructor
            %   ekf = EKFLocalizer(x0, P0, landmarks, params)
            %
            %   Inputs:
            %       x0        - [3×1] Initial state [x; y; theta]
            %       P0        - [3×3] Initial covariance
            %       landmarks - [N×3] Known landmarks [id, x, y]
            %       params    - Struct with fields:
            %                   .motionModel  - 'deadreckoning' or 'odometry'
            %                   .dt           - Time step (for dead reckoning)
            %                   .alpha        - Motion noise parameters
            %                   .R            - Measurement noise [2×2]
            %                   .sensorParams - .maxRange, .fov
            
            obj.mu = x0;
            obj.Sigma = P0;
            obj.landmarks = landmarks;
            obj.params = params;
        end
        
        function predict(obj, u)
            %PREDICT EKF prediction step
            %   ekf.predict(u)
            %
            %   Propagates state and covariance using motion model
            %
            %   Input:
            %       u - Control input
            %           Dead Reckoning: [2×1] [v; w]
            %           Odometry: [6×1] [x; y; th; x'; y'; th']
            
            % Propagate state using motion model
            if strcmp(obj.params.motionModel, 'deadreckoning')
                % Dead reckoning motion model
                current_mu = obj.mu;
                obj.mu = sampleDeadReckoningMotionModel(current_mu, u, obj.params.alpha, obj.params.dt);

                % Compute motion model Jacobian
                G = obj.computeMotionJacobianDR(current_mu, u, obj.params.dt);
                
                % Process noise covariance
                Q = obj.computeProcessNoiseDR(u, obj.params.alpha, obj.params.dt);
                
            elseif strcmp(obj.params.motionModel, 'odometry')
                % Odometry motion model
                current_mu = obj.mu;
                obj.mu = sampleOdometryMotionModel(current_mu, u, obj.params.alpha);

                % Compute motion model Jacobian
                G = obj.computeMotionJacobianOdom(current_mu, u);
                
                % Process noise covariance
                Q = obj.computeProcessNoiseOdom(u, obj.params.alpha);
                
            else
                error('Unknown motion model: %s', obj.params.motionModel);
            end
            
            % Wrap angle to [-pi, pi]
            obj.mu(3) = wrapToPi(obj.mu(3));
            
            % Propagate covariance
            obj.Sigma = G * obj.Sigma * G' + Q;
        end
        
        function update(obj, z_actual, detectedLandmarks)
            %UPDATE EKF measurement update step
            %   ekf.update(z_actual, detectedLandmarks)
            %
            %   Corrects state estimate using landmark measurements
            %
            %   Inputs:
            %       z_actual          - [2K×1] Measurements [r1;b1;r2;b2;...]
            %       detectedLandmarks - [K×3] Detected landmarks [id,x,y]
            
            % Check if we have measurements
            if isempty(z_actual) || isempty(detectedLandmarks)
                % No measurements, skip update
                return;
            end
            
            % Predict measurements for all landmarks
            [z_pred, H, validLandmarks] = predictMeasurement(...
                obj.mu, obj.landmarks, obj.params.sensorParams);
            
            % Associate actual detections with predictions
            [z_actual_matched, z_pred_matched, H_matched, R_matched] = ...
                associateMeasurements(detectedLandmarks, ...
                struct('ranges', z_actual(1:2:end), 'bearings', z_actual(2:2:end)), ...
                validLandmarks, z_pred, H, obj.params.R);
            
            % Check if we have matched measurements
            if isempty(z_actual_matched)
                % No matching landmarks, skip update
                return;
            end
            
            % Innovation (measurement residual)
            innovation = z_actual_matched - z_pred_matched;
            
            % Wrap bearing errors to [-pi, pi]
            innovation(2:2:end) = wrapToPi(innovation(2:2:end));
            
            % Innovation covariance
            S = H_matched * obj.Sigma * H_matched' + R_matched;
            
            % Kalman gain
            K = obj.Sigma * H_matched' / S;
            
            % State update
            obj.mu = obj.mu + K * innovation;
            
            % Wrap angle
            obj.mu(3) = wrapToPi(obj.mu(3));
            
            % Covariance update (Joseph form for numerical stability)
            I = eye(3);
            obj.Sigma = (I - K * H_matched) * obj.Sigma * (I - K * H_matched)' + ...
                        K * R_matched * K';
            
            % Ensure symmetry
            obj.Sigma = (obj.Sigma + obj.Sigma') / 2;
        end
        
        function [x_est, P_est] = getState(obj)
            %GETSTATE Get current state estimate and covariance
            %   [x_est, P_est] = ekf.getState()
            %
            %   Outputs:
            %       x_est - [3×1] State estimate [x; y; theta]
            %       P_est - [3×3] Covariance matrix
            
            x_est = obj.mu;
            P_est = obj.Sigma;
        end
        
        function P = getCovariance(obj)
            %GETCOVARIANCE Get current covariance matrix
            %   P = ekf.getCovariance()
            
            P = obj.Sigma;
        end
        
        function setState(obj, mu, Sigma)
            %SETSTATE Set state estimate and covariance
            %   ekf.setState(mu, Sigma)
            %
            %   Inputs:
            %       mu    - [3×1] State estimate
            %       Sigma - [3×3] Covariance matrix
            
            obj.mu = mu;
            obj.mu(3) = wrapToPi(obj.mu(3));
            obj.Sigma = Sigma;
        end
    end
    
    methods (Access = private)
        %% Private helper methods for Jacobians and noise
        
        function G = computeMotionJacobianDR(obj, x, u, dt)
            %COMPUTEMOTIONJACOBIANDR Jacobian of dead reckoning model
            
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
        
        function Q = computeProcessNoiseDR(obj, u, alpha, dt)
            %COMPUTEPROCESSNOISEDR Process noise for dead reckoning
            
            v = u(1);
            w = u(2);
            
            % Velocity variances
            var_v = alpha(1)*abs(v) + alpha(2)*abs(w);
            var_w = alpha(3)*abs(v) + alpha(4)*abs(w);
            var_gamma = alpha(5)*abs(v) + alpha(6)*abs(w);
            
            % Process noise in control space
            M = diag([var_v, var_w, var_gamma]) * dt^2;
            
            % Map to state space (simplified)
            Q = M;
        end
        
        function G = computeMotionJacobianOdom(obj, x, u)
            %COMPUTEMOTIONJACOBIANODOM Jacobian of odometry model
            
            theta = x(3);
            
            % Unpack odometry
            x_bar = u(1);
            y_bar = u(2);
            th_bar = u(3);
            x_bar_p = u(4);
            y_bar_p = u(5);
            
            % Compute deltas
            delta_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar) - th_bar;
            delta_trans = sqrt((x_bar - x_bar_p)^2 + (y_bar - y_bar_p)^2);
            delta_rot1 = wrapToPi(delta_rot1);
            
            % Jacobian
            G = eye(3);
            G(1, 3) = -delta_trans * sin(theta + delta_rot1);
            G(2, 3) = delta_trans * cos(theta + delta_rot1);
        end
        
        function Q = computeProcessNoiseOdom(obj, u, alpha)
            %COMPUTEPROCESSNOISEODOM Process noise for odometry
            
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
    end
end
