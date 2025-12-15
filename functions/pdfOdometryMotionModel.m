function p = pdfOdometryMotionModel(x, xp, u, alpha)
%MOTIONMODELODOMETRYPDF  p(x' | x, u, α) for Thrun odometry model
%
%   x   : [3×1]   previous pose   [x;  y;  θ]
%   xp  : [3×1]   current pose    [x'; y'; θ']
%   u   : [6×1]   odom poses in odom frame:
%                 u = [xbar; ybar; thetabar; xbar_p; ybar_p; thetabar_p]
%   alpha : [4×1] noise parameters [α1…α4]
%
%   p   : scalar likelihood value

    % unpack world poses
    x0 = x(1);  y0 = x(2);  th0 = x(3);
    x1 = xp(1); y1 = xp(2); th1 = xp(3);

    % unpack odometry poses (bar variables)
    xb   = u(1); yb   = u(2); thb   = u(3);
    xb_p = u(4); yb_p = u(5); thb_p = u(6);

    %% 1) Ideal deltas from odometry  (δ terms)
    delta_trans  = sqrt((xb_p - xb)^2 + (yb_p - yb)^2);
    delta_rot1   = wrapToPi(atan2(yb_p - yb, xb_p - xb) - thb);
    delta_rot2   = wrapToPi(thb_p - thb - delta_rot1);

    %% 2) Deltas implied by (x, x')  (δ̂ terms)
    dtrans_hat = sqrt((x1 - x0)^2 + (y1 - y0)^2);
    drot1_hat  = wrapToPi(atan2(y1 - y0, x1 - x0) - th0);
    drot2_hat  = wrapToPi(th1 - th0 - drot1_hat);

    %% 3) Errors (ensure angular wrap)
    e_rot1  = wrapToPi(delta_rot1  - drot1_hat);
    e_trans = delta_trans - dtrans_hat;
    e_rot2  = wrapToPi(delta_rot2  - drot2_hat);

    %% 4) Std deviations (Thrun noise model)
    % note: |·| used as in your slide
    sigma1 = sqrt(alpha(1)*abs(drot1_hat) + alpha(2)*abs(dtrans_hat) + eps);
    sigma2 = sqrt(alpha(3)*abs(dtrans_hat) + ...
                  alpha(4)*(abs(drot1_hat) + abs(drot2_hat)) + eps);
    sigma3 = sqrt(alpha(1)*abs(drot2_hat) + alpha(2)*abs(dtrans_hat) + eps);

    %% 5) Probabilities
    p1 = gaussianPDF(e_rot1,  sigma1);
    p2 = gaussianPDF(e_trans, sigma2);
    p3 = gaussianPDF(e_rot2,  sigma3);

    %% 6) Final likelihood
    p = p1 * p2 * p3;
end


function p = gaussianPDF(e, sigma)
% Normal pdf N(0, sigma^2) evaluated at error e
    p = (1/(sqrt(2*pi)*sigma)) * exp(-0.5*(e/sigma)^2);
end


%% Sample use
% x    = [0; 0; 0];
% xp   = [0.95; 0.05; 0.10];
% u    = [0;0;0; 1;0;0.1];          % odometry poses
% alpha = [0.1 0.1 0.1 0.1]';
% 
% p = pdfOdometryMotionModel(x, xp, u, alpha)

