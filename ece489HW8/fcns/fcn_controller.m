function tau = fcn_controller(t,X,p_hat,traj)
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
% p_hat: estimated robot parameter struct
% traj: struct containing the trajectory times and coefficients
%   traj.ts: vector of times in seconds
%   traj.coeffs: array of coefficients (various rad rates per time)
%
% OUTPUTS
% tau: 3x1 vector of joint torques in N m

%Robot parameters
params = p_hat.params;

%Current states 
q = X(1:3);     %joint angles
dq = X(4:6);    %joints velocities

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIGN YOUR CONTROLLER HERE!

%Compute desired trajecotry
[xd, xd_dot, xd_dotdot] = fcn_trajectory(t,traj);

% compute the actual task space position and velocities
x = fcn_p4(q, params); %% end effector position
J = fcn_J4(q, params); % endeffector jacobian to get the velocity
x_dot = J*dq; 
%Controller
%PD -- get error < 0.005m
%{
Kp = diag([2500; 1500; 7500]);
Kd = diag([140; 70; 300]);
G = fcn_Ge(q, params); %gravity comp
tau = J.'*(Kp*(xd - x) + Kd*(xd_dot - x_dot)) + G;
%}

% Task-space inverse dynamics control (Feedback linearizing control)
% M = fcn_De(q, params); % get the mass matrix
% C = fcn_Ce(q, dq, params); % Coriolis matrix
% G = fcn_Ge(q, params);          % gravity vector
% J_dot = fcn_dJ4(q, dq, params); % derivative of J
% 
% Kp = diag([3000; 2000; 5000]);
% Kd = diag([50; 100; 100]);
% 
% %compute acceleration in task-space
% ax = xd_dotdot + Kp*(xd - x) + Kd*(xd_dot - x_dot);
% 
% J_inv = pinv(J); % take the pseudo inverse of J 
% % Convert task space acceleraiton to joint acceleration
% q_dotdot = J_inv * (ax - J * dq);
% 
% % Final Torque Command
% tau = M *q_dotdot + C*dq + G;

% Admittance Control
Md = diag([1;1;1]);
Bd = diag([1;20;1]);
Kd = diag([100;100;100]);

[M, C, G] = task_space_EoM(t, X, p_hat);
a = xd_dotdot + pinv(Md)*(-Bd*(x_dot-xd_dot)-Kd*(x-xd));
tau = J.'*(M*a + C*x_dot + G);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control saturation - DO NOT CHANGE
tau(~isnumeric(tau) | ~isfinite(tau) | ~isreal(tau)) = 0; % enforce valid numbers
if any(~isnumeric(tau) | ~isfinite(tau) | ~isreal(tau))
    disp('Invalid commands');
end

taumax = 25; %Maximum torque in [Nm] that can be applied at each joint
wfree = 4; % free running speed
tauPlusMax = taumax*(1-dq/wfree);
tauMinusMax = taumax*(-1-dq/wfree);
for ii = 1:3
    if tau(ii) > taumax
        tau(ii) = taumax;
    end
    if tau(ii) < -taumax
        tau(ii) = -taumax;
    end
    if tau(ii) > tauPlusMax(ii)
        tau(ii) = tauPlusMax(ii);
    end
    if tau(ii) < tauMinusMax(ii)
        tau(ii) = tauMinusMax(ii);
    end
end


end