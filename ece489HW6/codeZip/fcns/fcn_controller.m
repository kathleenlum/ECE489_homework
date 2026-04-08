function tau = fcn_controller_sol(t,X,p)
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

%Calculating the matrices for the equation of motion:
De = fcn_De(q,params);      %Inertia metrix
Ce = fcn_Ce(q,dq,params);   %Coriolis matrix
Ge = fcn_Ge(q,params);      %Gravity vector

%Compute desired trajecotry
[qd, qd_dot, qd_dotdot] = fcn_trajectory(t);

%Controller
Kp = diag([30; 30; 30]);
Kd = diag([15; 15; 15]);

Kff = diag(diag(De));

% add Ge to tau to get torque to compensate gravity
% add feedforward 
tau = Kp*(qd - q) + Kd*(qd_dot - dq) + Ge + Kff*qd_dotdot;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control saturation - DO NOT CHANGE
taumax = 15; %Maximum torque in [Nm] that can be applied at each joint
tau(tau>taumax) = taumax;
tau(tau<-taumax) = -taumax;


end