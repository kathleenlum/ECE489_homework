function tau = fcn_controller_sol(t,X,p,traj)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Compute joint torque commands for the CRS robot arm simulation.  Write
% your control within the commented block.  The default value provided, 
% tau = [0; 0; 0], commands zero torque at all joints.
%
% INPUTS
% t: time in seconds (scalar)
% X: robot state vector: [q1; q2; q3; dq1; dq; dq3] in rad and rad/s
% p: robot parameter struct
%
% OUTPUTS
% tau: 3x1 vector of joint torques in N m

%Robot parameters
params = p.params;

%Current states 
q = X(1:3);     %joint angles
dq = X(4:6);    %joints velocities

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIGN YOUR CONTROLLER HERE!

%Compute desired trajecotry

[qd, qd_dot, qd_dotdot] = fcn_trajectory(t,traj);

M = fcn_De(q,params);

% PD Controller
%{
Kp = diag([1000; 1000; 1000]);
Kd = diag([500;500;500]);
tau = M*(Kp*(qd - q) + Kd*(qd_dot - dq));
%}

% Feedback linearizing controller
C = fcn_Ce(q,dq,params);
G = fcn_Ge(q,params);
Kp = diag([900; 900; 900]);
Kd = diag([100;100;100]);
tau = M*(qd_dotdot + Kp*(qd - q) + Kd*(qd_dot - dq)) + C*qd + G;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control saturation - DO NOT CHANGE
taumax = 20; %Maximum torque in [Nm] that can be applied at each joint
tau(tau>taumax) = taumax;
tau(tau<-taumax) = -taumax;


end