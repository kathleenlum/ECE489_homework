% 3-link manipulator simulation
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2024

addpath gen
addpath fcns

% parameters --------------------------------------------------------------
% Get the parameters about the robot and simulation settings
p = get_params();
params = p.params;

q0 = [0;0;0];
pd0 = [0.4;0;0.15];
pd1 = [0.2; 0;0.15];
pd2 = [0.2;0;0.55];
pd3 = [0.4; 0; 0.55];
% compute the inverse kinematics for the four desired joint configurations
qd0 = CRS_IK(pd0, q0);
qd1 = CRS_IK(pd1, qd0);
qd2 = CRS_IK(pd2, qd1);
qd3 = CRS_IK(pd3, qd2);

display(qd0);
display(qd1);
display(qd2);
display(qd3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up trajectory variables here ----------------------------------------
ts = [0; 1; 2; 3]; % waypoint times (s)

q_ds = [qd0,qd1,qd2,qd3]; % waypoint locations in joint coordinates (rad)
% Each row is a joint (1 to 3) and each column is a waypoint.

coeffs = poly_traj_coeffs(q_ds, ts);

% traj is a struct containing the trajectory times and coefficients that
% you solve for so that they can be used during the simulation
traj.ts = ts;
traj.coeffs = coeffs;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions -------------------------------------------------------
%Initial conditions:
q0 = qd0; %[0; 0; 0];
dq0 = [0; 0; 0];
ic = [q0; dq0];

%Plot the robot at initial conditions:
% figure
% plotRobot(ic,p,[0 0.5 1]);

% Simulation --------------------------------------------------------------
tfinal = 3;
[t,X] = ode45(@(t,X)dyn_manip(t,X,p,traj),[0 tfinal],ic);
X = X.'; % Transpose the state vector so that it is 3xn instead of nx3

%Reconstructing control inputs and desired trajectory for plotting
tau = zeros(3,length(t));
qd = zeros(3,length(t));
qd_dot = zeros(3,length(t));
qd_dotdot = zeros(3,length(t));
for ii = 1:length(t)
    tau(:,ii) = fcn_controller(t(ii),X(:,ii),p,traj);
    [qd(:,ii),  qd_dot(:,ii),  qd_dotdot(:,ii) ] = fcn_trajectory(t(ii),traj);
end

% Plotting ----------------------------------------------------------------
%Plotting robot trajectories
figure(1)
clf

subplot(3,1,1)
plot(t,X(1:3,:),'-','LineWidth',1.5)
hold on
set(gca,'ColorOrderIndex',1)
plot(t,qd,':','LineWidth',2)
hold off
ylabel('Angles q [rad]')
legend('\theta_1','\theta_2','\theta_3','\theta_{d1}','\theta_{d2}','\theta_{d3}')
grid on

subplot(3,1,2)
plot(t,X(4:6,:),'-','LineWidth',1.5)
hold on
set(gca,'ColorOrderIndex',1)
plot(t,qd_dot,':','LineWidth',2)
hold off
ylabel('Velocity dq/dt [rad/s]')
%legend('d\theta_1/dt','d\theta_2/dt','d\theta_3/dt',...
%    'd\theta_{d1}/dt','d\theta_{d2}/dt','d\theta_{d3}/dt')
grid on

subplot(3,1,3)
plot(t,tau,'LineWidth',1.5)
ylabel('Joint torque \tau [Nm]')
xlabel('Time [s]')
legend('\tau_1','\tau_2','\tau_3')
grid on
pos = get(gcf,'position');
set(gcf,'position',[pos(1),pos(2)+pos(4)-600,560,600])

figure(2)
clf
plot(t,X(1:3,:)-qd,'LineWidth',2)
ylabel('Joing angle error e(t) = q_d(t) - q(t) [rad]')
xlabel('Time [s]')
legend('e_1','e_2','e_3')
grid on

figure(3)
clf
subplot(3,1,1)
plot(t,qd,':','Linewidth',2)
legend('\theta_{d1}','\theta_{d2}','\theta_{d3}')
ylabel('Desired angles [rad]')
grid on
subplot(3,1,2)
plot(t,qd_dot,':','Linewidth',2)
ylabel('Desired velocities [rad/s]')
grid on
subplot(3,1,3)
plot(t,qd_dotdot,':','Linewidth',2)
ylabel('Desired accelerations [rad/s^2]')
grid on
xlabel('Time [s]')
pos = get(gcf,'position');
set(gcf,'position',[pos(1),pos(2)+pos(4)-600,560,600])

%Animating the robot
animateRobot(t,X,p,0)










