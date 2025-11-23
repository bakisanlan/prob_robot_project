function p = pdfDeadReckoningMotionModel(x, xp, u, alpha, dt)
%MOTIONMODELDEADRECKONINGPDF
%   Computes the probability density p(x' | x, u, α)
%   according to Thrun's dead-reckoning inverse motion model.
%
%   x   = [x; y; theta]      (current pose)
%   xp  = [x'; y'; theta']   (observed next pose)
%   u   = [v; w]             (commanded translational & rotational velocity)
%   alpha = [α1 ... α6]'     (noise params)
%   dt  = time step
%
%   Returns:
%       p   = scalar pdf value

    % unpack variables
    x0 = x(1);   y0 = x(2);   th0 = x(3);
    x1 = xp(1);  y1 = xp(2);  th1 = xp(3);
    v  = u(1);   w  = u(2);

    %% Step 1: Compute µ (Thrun Trick)
    mu = 0.5 * ((x0 - x1)*cos(th0) + (y0 - y1)*sin(th0)) / ...
              ((y0 - y1)*cos(th0) - (x0 - x1)*sin(th0));

    %% Step 2: Compute auxiliary midpoint (x*, y*)
    x_star = (x0 + x1)/2 + mu*(y0 - y1);
    y_star = (y0 + y1)/2 + mu*(x1 - x0);

    %% Step 3: Radius r*
    r_star = sqrt((x0 - x_star)^2 + (y0 - y_star)^2);

    %% Step 4: Rotation delta theta
    dtheta = atan2(y1 - y_star, x1 - x_star) - ...
             atan2(y0 - y_star, x0 - x_star);

    %% Step 5: Recover "true" executed controls
    v_hat = (dtheta / dt) * r_star;
    w_hat = dtheta / dt;
    gamma_hat = (th1 - th0)/dt - w_hat;

    %% Step 6: Noise variances
    var_v = alpha(1)*abs(v) + alpha(2)*abs(w);
    var_w = alpha(3)*abs(v) + alpha(4)*abs(w);
    var_g = alpha(5)*abs(v) + alpha(6)*abs(w);

    sig_v = sqrt(var_v + eps);
    sig_w = sqrt(var_w + eps);
    sig_g = sqrt(var_g + eps);

    %% Step 7: Gaussian pdf values
    pv = gaussianPDF(v - v_hat, sig_v);
    pw = gaussianPDF(w - w_hat, sig_w);
    pg = gaussianPDF(gamma_hat,   sig_g);

    %% Step 8: Final likelihood
    p = pv * pw * pg;
end


function p = gaussianPDF(error, sigma)
% Normal distribution N(0, sigma^2)
    p = (1/(sqrt(2*pi)*sigma)) * exp(-0.5*(error/sigma)^2);
end

%% Sample use
% x   = [0; 0; 0];
% xp  = [0.49; 0.01; 0.10];
% u   = [0.5; 0.2];
% dt  = 0.1;
% alpha = [0.1 0.1 0.1 0.1 0.05 0.05]';
% 
% p = pdfDeadReckoningMotionModel(x, xp, u, alpha, dt)

