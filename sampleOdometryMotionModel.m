function x_next = sampleOdometryMotionModel(x, u, alpha)
%SAMPLEMOTIONMODOLOGY  Odometry-based motion model (sampling version)
%
%   x_next = sampleMotionModelOdometry(x, u, alpha)
%
%   Inputs
%   ------
%   x     : [3×1] current pose of robot        [x; y; theta]
%   u     : [6×1] odometry reading
%           u = [x_bar; y_bar; theta_bar; x_bar_p; y_bar_p; theta_bar_p]
%             (pose at time t-1 and pose at time t in odometry frame)
%   alpha : [4×1] noise parameters α1…α4
%
%   Output
%   ------
%   x_next : [3×1] sampled new pose [x'; y'; theta']

    % unpack robot pose
    xr  = x(1);  %r stansts for robot which current pose
    yr  = x(2);
    thr = x(3);

    % unpack odometry poses
    x_bar    = u(1);
    y_bar    = u(2);
    th_bar   = u(3);
    x_bar_p  = u(4);  % _p is for prime which correspondes to next value
    y_bar_p  = u(5);
    th_bar_p = u(6);

    %% 1) Ideal odometry-based motion (δ_rot1, δ_trans, δ_rot2)
    delta_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar) - th_bar;
    delta_trans = sqrt((x_bar - x_bar_p)^2 + (y_bar - y_bar_p)^2);
    delta_rot2 = th_bar_p - th_bar - delta_rot1;

    % normalize angles
    delta_rot1 = wrapToPi(delta_rot1);
    delta_rot2 = wrapToPi(delta_rot2);

    %% 2) Add noise according to Thrun's model
    % variance terms (standard textbook form)
    var_rot1  = alpha(1)*delta_rot1 + alpha(2)*delta_trans;
    var_trans = alpha(3)*delta_trans + alpha(4)*(delta_rot1 + delta_rot2);
    var_rot2  = alpha(1)*delta_rot2 + alpha(2)*delta_trans;

    std_rot1  = sqrt(var_rot1 + eps);
    std_trans = sqrt(var_trans + eps);
    std_rot2  = sqrt(var_rot2 + eps);

    % noisy versions (δ̂ terms)
    delta_rot1_hat  = delta_rot1  - std_rot1  * randn;
    delta_trans_hat = delta_trans - std_trans * randn;
    delta_rot2_hat  = delta_rot2  - std_rot2  * randn;

    %% 3) Propagate robot pose
    xr_p  = xr + delta_trans_hat * cos(thr + delta_rot1_hat);
    yr_p  = yr + delta_trans_hat * sin(thr + delta_rot1_hat);
    thr_p = thr + delta_rot1_hat + delta_rot2_hat;

    thr_p = wrapToPi(thr_p);

    x_next = [xr_p; yr_p; thr_p];
end


%% Sample use
% x = [0; 0; 0];   % world pose
% u = [0;0;0; 1;0;0.1];        % odometry from t-1 to t
% alpha = [0.1 0.1 0.1 0.1]';  % tune as needed
% 
% x1 = sampleOdometryMotionModel(x, u, alpha);

