function tau = fcn_controller(t,X,p_hat)

%Robot parameters
%You are not allowed to call the function get_params();
params = p_hat.params;

%Current states 
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector

%Actuators reflected inertia matrix:
N = 25;
Ir = 1.4e-5;
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir]];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIGN YOUR CONTROLLER HERE!
Ge = fcn_Ge(q,params);      %Gravity vector
Ce = fcn_Ce(q,dq,params);   %Coriolis vector
De = fcn_De(q,params)+M_DH; %Inertia matrix
x = fcn_p4(q,params);       %End-effector position
J = fcn_J4(q,params);       %End-effector Jacobian
dx = J*dq;                  %End-effector velocity

d = 0.0673; % distance from wrist to bottom of rod
h = 6*0.0254; % total rod height
T0_1 = fcn_T01(q, params);
R0_1 = T0_1(1:3, 1:3);
% have to add bottom of rod offset relative to the direction robot joint 1
% p1 = [0; 0.4; 0.35];
%p1_modified = R01' * (R01*p1 + [d; 0; h/2 + 0.01]);
% pd = fcn_p4(q,params) + R0_1*[d; 0; -0.5*h]; 
p_curr = x + R0_1*[d; 0; -0.5*h]; %position of bottom of rod
dp_curr = J*dq;

% Gains (Must be tuned to stay within +/- 5mm)
Kp = diag([1000, 1000, 1000]); 
Kd = diag([100, 100, 100]);

%%%%%%%%%%%%%
% commanded trajectory function:
% [pd, pd_dot] = fcn_trajectory(t, p_hat);
% qd = xd: 3x1 desired Cartesian position [x; y; z] in meters
% qd_dot = xd_dot: 3x1 desired Cartesian velocity [vx; vy; vz] in m/s
%Waypoints
P_des = [0,    0,   0,    0.35,  0.35,  0,    0;
         0.4,  0.4, 0.4,  0.15, -0.15, -0.4, -0.4;
         0.35, 0.1, 0.05, 0.15,  0.15,  0.4,  0.3];

%Desired trajectory
% Default velocity and force unless otherwise specified
pd = [0; 0; 0]; %Desired joint angles at time t
pd_dot = [0; 0; 0]; %Desired joint velocities at time t
% parse through every waypoint for each timestep of 1s

% part 1A: Peg Insertion
if t<0.5
    pd = [0.3;0.3;0.35];
elseif t < 1 % p1 Hover
    %First target point
    pd = [0;0.4;0.35];
    
elseif t<=2 % p2 Surface
    pd = [0; 0.4; 0.1];
    
elseif t<=3 % p3 Insert
    pd = [0; 0.4; 0.05];
    Kp = diag([1000, 1000, 1000]); 
    Kd = diag([100, 250, 100]);
elseif t<=4.5 % p2 Return/pull out of hole
    pd = [0; 0.4; 0.1+ 0.03];
    Kp = diag([1000, 1000, 1000]); 
    Kd = diag([100, 200, 300]);
% Narrow Slot
elseif t<=5.5 % p4
    pd = [0.35+0.01; 0.15; 0.15];
    %Kd = diag([300, 100, 100]);
    Kp = diag([800, 800, 800]); 
    Kd = diag([200, 200, 200]);
elseif t<=7 % p5
    pd = [0.35; -0.15; 0.15];
    % Kp = diag([2000, 1000, 1000]); 
    % Kd = diag([500, 100, 100]);
    Kp = diag([1200, 800, 1200]); % High X/Z to stay in track, low Y to slide
    Kd = diag([250, 150, 250]);
% elseif t<=9.5 % p6 hover
%     pd = [0; -0.4; 0.4];
%     Kp = diag([1000, 1000, 1000]); 
%     Kd = diag([100, 100, 150]);
% elseif t<=11.25 %p7
%     pd = [0; -0.4; 0.3 - 0.04]; % tune z axis to apply more pressure
%     Kp = diag([1000, 1000, 400]); 
%     Kd = diag([100, 100, 150]);
%     pd = [0; -0.4; 0.3]; % The actual surface height
% 
%     1. Calculate standard PD Force for all axes
%     We will override Z in the next step
%     F_cmd = Kp*(pd - p_curr) + Kd*(pd_dot - dp_curr);
% 
%     2. OVERRIDE Z: Set Position Gain to 0 and Force to 10.5N
%     We use 10.5N to ensure the "steady state" stays above the 10N threshold
%     F_cmd(3) = -10.5 + 200 * (0 - dp_curr(3)); % Force + Damping
% elseif t<=11.25 %p7
%     pd = [0; -0.4; 0.3];
%     Kp = diag([1000, 1000, 1500]); 
%     Kd = diag([100, 100, 150]);
% elseif t<=11.75 %p6
%     pd = [0; -0.4; 0.4];
% else
%     pd = [0; -2.63; 2.13] + R0_1*[d; 0; -0.5*h];
% end
% 
% 
% %%%%%%%%%%%%
% 
% calculate joint torque commands
% error = pd- p_curr;
% error_dot = pd_dot - dp_curr;
% F_cmd = Kp*error + Kd*error_dot;
% tau = J' * F_cmd + Ce*dq + Ge;%[0;0;0];

elseif t <= 8 % p6 Hover (Settling point)
    pd = [0; -0.4; 0.4];
    Kp = diag([1000, 1000, 1000]); 
    Kd = diag([150, 150, 30]);
    % We let the code flow to the bottom calculation for this part

elseif t <= 10 % p7 Force Task control 
    pd = [0; -0.4; 0.3];
    Kp = diag([800, 800, 1800]); 
    Kd = diag([80, 80, 600]);
    % calculate x and y force normally
    F_cmd_xy = Kp(1:2,1:2)*(pd(1:2) - p_curr(1:2)) + Kd(1:2,1:2)*(pd_dot(1:2) - dp_curr(1:2));
    
    % make z force hit 10N + damping (no position error)
    F_z_hybrid = -11.0 - 300 * dp_curr(3);
    F_cmd = [F_cmd_xy; F_z_hybrid];
    
    % IMPORTANT: Skip the bottom PD calculation
    tau = J' * F_cmd + Ce*dq + Ge;
elseif t <= 10.5 % p6 get out of hole
    pd = [0; -0.4; 0.4];
    Kp = diag([1000, 1000, 1300]); 
    Kd = diag([150, 150, 600]);
    % We let the code flow to the bottom calculation for this part

else % Default/Hold
    starting = [0; -2.63; 2.13]; 
    pd = fcn_p4(starting,params) + R0_1*[d; 0; -0.5*h]; 
end

%hybrid controller 
if t > 8.5 && t <= 10
    % tau is already calculated for commanded force control
else
    % calculate joint torque commands for standard task position
    error = pd - p_curr;
    error_dot = pd_dot - dp_curr;
    F_cmd = Kp*error + Kd*error_dot;
    tau = J' * F_cmd + Ce*dq + Ge;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control saturation - DO NOT CHANGE
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