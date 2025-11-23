function x_next = GTodometryMotionModel(x, u)
%MOTIONMODELODOMETRYGT  Noise-free odometry motion model
%
%   x_next = motionModelOdometryGT(x, u)
%
%   x     : [3×1] current world pose [x; y; theta]
%   u     : [6×1] odometry poses in odom frame
%           u = [xbar; ybar; thetabar; xbar_p; ybar_p; thetabar_p]
%
%   x_next: [3×1] next world pose [x'; y'; theta']

    % unpack world pose
    xr  = x(1);
    yr  = x(2);
    thr = x(3);

    % unpack odometry poses (in odom frame)
    xb    = u(1); yb    = u(2); thb    = u(3);
    xb_p  = u(4); yb_p  = u(5); thb_p  = u(6);

    % --- ideal odometry deltas (no noise) -------------------------------
    delta_trans = sqrt((xb_p - xb)^2 + (yb_p - yb)^2);
    delta_rot1  = atan2(yb_p - yb, xb_p - xb) - thb;
    delta_rot1  = wrapToPi(delta_rot1);
    delta_rot2  = thb_p - thb - delta_rot1;
    delta_rot2  = wrapToPi(delta_rot2);

    % --- propagate robot pose in world frame ----------------------------
    xr_p  = xr + delta_trans * cos(thr + delta_rot1);
    yr_p  = yr + delta_trans * sin(thr + delta_rot1);
    thr_p = thr + delta_rot1 + delta_rot2;
    thr_p = wrapToPi(thr_p);

    x_next = [xr_p; yr_p; thr_p];
end

%% Sample use
% x = [0; 0; 0];
% u = [0;0;0; 1;0;0.1];   % odometry from t-1 to t
% x1 = GTodometryMotionModel(x, u);

