function x_next = sampleDeadReckoningMotionModel(x, u, alpha, dt, varargin)
%SAMPLEMOTIONMODELVELOCITY  Thrun-style velocity motion model (6 alphas)
%
%   x_next = sampleMotionModelVelocity(x, u, alpha, dt)
%   x_next = sampleMotionModelVelocity(x, u, alpha, dt, occMap)
%
%   x      : [3×1] current pose [x; y; theta]
%   u      : [2×1] control [v; w]  (translational & rotational velocity)
%   alpha  : [6×1] noise parameters (α1…α6)
%   dt     : scalar time step
%   occMap : (optional) occupancyMap object for collision checking
%
%   x_next : [3×1] new pose [x'; y'; theta'], or [NaN; NaN; NaN] if in collision

    % unpack state and control
    px    = x(1);
    py    = x(2);
    theta = x(3);

    v = u(1);
    w = u(2);

    % --- 1) sample noisy velocities (Thrun et al. style) -----------------
    % sample(b) ~ N(0, b^2)
    std_v     = sqrt(alpha(1)*abs(v) + alpha(2)*abs(w));
    std_w     = sqrt(alpha(3)*abs(v) + alpha(4)*abs(w));
    std_gamma = sqrt(alpha(5)*abs(v) + alpha(6)*abs(w));

    v_hat     = v + std_v*randn;
    w_hat     = w + std_w*randn;
    gamma_hat = std_gamma*randn;

    % --- 2) propagate pose with dead reckoning ---------------------------
    % handle small angular velocity separately to avoid division by zero
    eps_w = 1e-6;

    if abs(w_hat) > eps_w
        % general case (circular arc)
        r = v_hat / w_hat;

        px_p = px - r * sin(theta) + r * sin(theta + w_hat*dt);
        py_p = py + r * cos(theta) - r * cos(theta + w_hat*dt);
        th_p = theta + w_hat*dt + gamma_hat*dt;
    else
        % nearly straight line motion
        px_p = px + v_hat*dt*cos(theta);
        py_p = py + v_hat*dt*sin(theta);
        th_p = theta + gamma_hat*dt;
    end

    % wrap angle to [-pi, pi] (optional but usually convenient)
    th_p = wrapToPi(th_p);

    x_next = [px_p; py_p; th_p];
    
    %% Check collision with occupancy map (if provided)
    if ~isempty(varargin) && ~isempty(varargin{1})
        occMap = varargin{1};
        % Check if the new position is in occupied space
        if checkOccupancy(occMap, [px_p, py_p])
            % Position is occupied, return NaN to indicate invalid sample (will not be plotted)
            x_next = [NaN; NaN; NaN];
            return;
        end
    end
end


%% Sample use
% x     = [0; 0; 0];              % initial pose
% u     = [0.5; 0.2];             % v = 0.5 m/s, w = 0.2 rad/s
% dt    = 0.1;                    % 100 ms step
% alpha = [0.1 0.1 0.1 0.1 0.01 0.01]';   % tune these
% 
% x1 = sampleDeadReckoningMotionModel(x, u, alpha, dt);
% x2 = sampleDeadReckoningMotionModel(x, u, alpha, dt, occMap);

