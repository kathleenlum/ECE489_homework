function tau = fcn_controller(t,X,p,contact)
% t: time in seconds
% X: robot state vector (rad and rad/s)
% p: parameter vector
% contact: whether the end effector is in contact (1) or not (0)
% tau: joint torques in N m

%Robot parameters
params = p.params;

%Current states 
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIGN YOUR CONTROLLER HERE!
J  = fcn_J4(q, params);
Jt = pinv(J);
Jwall = fcn_Jwall(q);
C = fcn_Ce(q, dq, params);
G = fcn_Ge(q, params);

%tau = [0; 0; 0];

%{
% Force Controller 
% Desired force matrix
lambda_d = [10; 0; 0]; %[Fx; Fy; Fz]
% calculate friction
fric = diag(p.friction(1:3, 1))*tanh(20*dq) + diag(p.friction(4:6, 1))*dq;
% torque calculation for force controller
% u = J.'*force + Coriolis + Gravity
tau = J.' *lambda_d + C*dq + G + fric;
%}

%{ asdfsadf
% Hybrid force-position controller (with gravity and Coriolis compensation)
%essentially building off of the force controller
[Xd, Xd_dot, Xd_dotdot] = fcn_trajectory(t);

%current position of end effector in task space coordinates
curr_X = fcn_p4(q,params);
curr_Xdot = J*dq;

% Desired force matrix
lambda_d = [0; 0; 0];

% calculate friction
fric = diag(p.friction(1:3, 1))*tanh(20*dq) + diag(p.friction(4:6, 1))*dq;

if(contact == 0) % no contact with wall
    % no contact gains
    Kp = diag([150; 300; 500]);
    Kd = diag([50; 150; 150]);
    F = Xd_dotdot + Kd*(Xd_dot - curr_Xdot) + Kp*(Xd - curr_X);
    lambda_d = [F(1); F(2); F(3)];
else
    % task space error
    vel_error = Xd_dot - curr_Xdot;
    pos_error = Xd - curr_X;

    % contact gains
    Kp = 1000;%diag([1000; 1000]); 
    Kd = 100;%diag([100; 120]);
    % follow the force trajectory for y and z
    F = Xd_dotdot(2:3) + Kd*(vel_error(2:3)) + Kp*(pos_error(2:3));
    % continue to apply 10N force in x direction
    lambda_d = [10; F(1); F(2)];
end

% torque calculation for force controller
% u = J.'*force + Coriolis + Gravity
tau = J.' *lambda_d + C*dq + G + fric;




%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control saturation - DO NOT CHANGE
taumax = 20; %Maximum torque in [Nm] that can be applied at each joint
tau(tau>taumax) = taumax;
tau(tau<-taumax) = -taumax;


end