function x_next = GTdeadReckoningMotionModel(x, u, dt)
%MOTIONMODELVELOCITYGT  Noise-free (ground-truth) velocity motion model
%
%   x_next = motionModelVelocityGT(x, u, dt)
%
%   x     : [3×1] current pose [x; y; theta]
%   u     : [2×1] control [v; w]  (translational, rotational velocity)
%   dt    : scalar time step
%
%   x_next: [3×1] next pose [x'; y'; theta']

    % unpack state and control
    px    = x(1);
    py    = x(2);
    theta = x(3);

    v = u(1);
    w = u(2);

    % small threshold to avoid division by w≈0
    eps_w = 1e-6;

    if abs(w) > eps_w
        % exact circular-arc integration
        r = v / w;

        px_p = px - r * sin(theta) + r * sin(theta + w*dt);
        py_p = py + r * cos(theta) - r * cos(theta + w*dt);
        th_p = theta + w*dt;
    else
        % straight-line limit case
        px_p = px + v*dt*cos(theta);
        py_p = py + v*dt*sin(theta);
        th_p = theta;
    end

    % wrap orientation to [-pi, pi]
    th_p = wrapToPi(th_p);

    x_next = [px_p; py_p; th_p];
end


%% Sample use
% x = [0; 0; 0];
% dt = 0.1;
% u = [0.5; 0.2];                % your commanded v,w
% X_gt = GTdeadReckoningMotionModel(x, u, dt);

