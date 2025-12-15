function x_next = GTodometryMotionModelDeltaInput(x, delta)
%MOTIONMODELODOMETRYDELTASGT  Noise-free odometry model with deltas
%
%   x_next = motionModelOdometryDeltasGT(x, delta)
%
%   x     : [3×1] current pose [x; y; theta]
%   delta : [3×1] [delta_rot1; delta_trans; delta_rot2]
%
%   x_next: [3×1] next pose [x'; y'; theta']

    % unpack pose
    xr  = x(1);
    yr  = x(2);
    thr = x(3);

    % unpack deltas
    delta_rot1  = delta(1);
    delta_trans = delta(2);
    delta_rot2  = delta(3);

    % propagate pose (noise-free Thrun odometry kinematics)
    xr_p  = xr + delta_trans * cos(thr + delta_rot1);
    yr_p  = yr + delta_trans * sin(thr + delta_rot1);
    thr_p = thr + delta_rot1 + delta_rot2;

    thr_p = wrapToPi(thr_p);   % local wrapper below

    x_next = [xr_p; yr_p; thr_p];
end


