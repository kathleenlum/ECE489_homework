function [qd, qd_dot, qd_dotdot] = fcn_trajectory(t)
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

A = 0.2; % rad/s
w = 10; % rad/s
%Desired trajectory
qd = [A*sin(w*t); A*cos(w*t)-A; A*sin(w*t)];%[-1; -pi/4; pi/3]; %Desired joint angles at time t
qd_dot = [A*w*cos(w*t); -A*w*sin(w*t); A*w*cos(w*t)]%[0; 0; 0]; %Desired joint velocities at time t
qd_dotdot = [-A*w*w*sin(w*t); -A*w*w*cos(w*t); -A*w*w*sin(w*t)];%[0; 0; 0]; %Desired joint accelerations at time t

end