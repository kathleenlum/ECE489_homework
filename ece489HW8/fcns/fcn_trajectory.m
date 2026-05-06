function [qd, qd_dot, qd_dotdot] = fcn_trajectory(t,q, traj)

% qd = xd: 3x1 desired Cartesian position [x; y; z] in meters
% qd_dot = xd_dot: 3x1 desired Cartesian velocity [vx; vy; vz] in m/s
% qd_dotdot = xd_dotdot: 3x1 desired Cartesian acceleration [ax; ay; az] in m/s^2

% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Specify a desired trajectory and its first and second derivatives.
%
% INPUTS:
% t: time in seconds (scalar)
%
% OUTPUTS:
% qd: 3x1 desired joint angle vector in rad at time t
% qd_dot: 3x1 desired joint velocity vector in rad/s at time t
% qd_dotdot: 3x1 desired joint acceleration vector in rad/s^2 at time t
%Desired trajectory
qd = [0; 0; 0]; %Desired joint angles at time t
qd_dot = [0; 0; 0]; %Desired joint velocities at time t
qd_dotdot = [0; 0; 0]; %Desired joint accelerations at time t
% Get the times and coefficients from the traj struct
ts = traj.ts;
coeffs = traj.coeffs;

% If statements for the time (t)
if t < 1.0
    %First target point
    qd = [0.4; 0; 0.15];
else
    %Second target point (Step jump occurs at t = 1s)
    qd = [0.2; 0.2; 0.35];
end

% If statements for the time (t)
% if t < 1.0
%     %First target point
%     qd = [0.4; 0; 0.15];
% else
%     %Second target point (Step jump occurs at t = 1s)
%     qd = [0.5; 0; 0.4];
% end

% if t <= ts(2)
%    % t0->t1
%    for j = 1:3
%        c = coeffs(1:5, j);
%        qd(j) = c(1) + c(2)*t + c(3)*t^2 + c(4)*t^3 + c(5)*t^4;
%        qd_dot(j) = c(2) + 2*c(3)*t + 3*c(4)*t^2 + 4*c(5)*t^3;
%        qd_dotdot(j) = 2*c(3) + 6*c(4)*t + 12*c(5)*t^2;
%    end
% 
% elseif t <= ts(3)
%    % t1->t2
%    for j = 1:3
%        c = coeffs(6:9, j);
%        qd(j) = c(1) + c(2)*t + c(3)*t^2 + c(4)*t^3;
%        qd_dot(j) = c(2) + 2*c(3)*t + 3*c(4)*t^2;
%        qd_dotdot(j) = 2*c(3) + 6*c(4)*t;
%    end 
% else
%    % Segment 3: 4th order (Coefficients 10 to 14)
%    % Clamp t to ts(4) to avoid divergence if ode45 steps slightly past tfinal
%    t_eval = min(t, ts(4));
% 
%    for j = 1:3
%        c = coeffs(10:14, j);
%        qd(j) = c(1) + c(2)*t_eval + c(3)*t_eval^2 + c(4)*t_eval^3 + c(5)*t_eval^4;
%        qd_dot(j) = c(2) + 2*c(3)*t_eval + 3*c(4)*t_eval^2 + 4*c(5)*t_eval^3;
%        qd_dotdot(j) = 2*c(3) + 6*c(4)*t_eval + 12*c(5)*t_eval^2;
%    end
% end

end


