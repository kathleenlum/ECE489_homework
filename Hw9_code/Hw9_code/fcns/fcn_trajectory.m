function [Xd, Xd_dot, Xd_dotdot] = fcn_trajectory(t)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Specify a desired trajectory and its first and second derivatives.
%
% INPUTS:
% t: time in seconds (scalar)
%
% OUTPUTS:
% Xd: 3x1 desired end effector location in m at time t
% Xd_dot: 3x1 desired velocity vector in m/s at time t
% Xd_dotdot: 3x1 desired acceleration vector in m/s^2 at time t

%Desired trajectory
Xd = [0.35; 0.1*sin(2*t); 0.45 + 0.05*cos(2*t)];%[0; 0; 0.5]; %Desired position at time t
Xd_dot = [0; 2*0.1*cos(2*t); -1*2*0.05*sin(2*t)];%[0; 0; 0]; %Desired velocities at time t
Xd_dotdot = [0; -2*2*0.1*sin(2*t); -2*2*0.05*cos(2*t)];%[0; 0; 0]; %Desired accelerations at time t

end