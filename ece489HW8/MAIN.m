% 3-link manipulator simulation
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023

addpath gen
addpath fcns

% parameters --------------------------------------------------------------
% Get the parameters about the robot and simulation settings
p = get_params();
params = p.params;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up trajectory variables here ----------------------------------------
ts = [0; 1; 2; 3]; % waypoint times (s)

% waypoint locations in task space (m)
q_ds = [0.4, 0.2, 0.2, 0.4;      % X positions
        0.0, 0.0, 0.0, 0.0;      % Y positions
        0.15, 0.15, 0.55, 0.55]; % Z positions 
% Each row is a joint (1 to 3) and each column is a waypoint.

coeffs = poly_traj_coeffs(q_ds, ts);

% traj is a struct containing the trajectory times and coefficients that
% you solve for so that they can be used during the simulation
traj.ts = ts;
traj.coeffs = coeffs;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions -------------------------------------------------------
%Initial conditions:
%These joint angles correspond to X = 0.4, Y = 0, Z = 0.15
q0 = [0; -0.3661; 1.241];%q_ds(:,1);%[0;0;0];
dq0 = [0; 0; 0];
ic = [q0; dq0];

%Plot the robot at initial conditions:
% figure
% plotRobot(ic,p,[0 0.5 1]);

% Simulation --------------------------------------------------------------
tfinal = 5;
[t,X] = Euler_Integrator(@(t,X)dyn_manip(t,X,p,traj),[0 tfinal],ic,4E-3);
X = X.'; % Transpose the state vector so that it is 3xn instead of nx3

%Reconstructing control inputs and desired trajectory for plotting
tau = zeros(3,length(t));
Xd = zeros(3,length(t));
Xd_dot = zeros(3,length(t));
Xd_dotdot = zeros(3,length(t));
Xe = zeros(3,length(t));
Xe_dot = zeros(3,length(t));
for ii = 1:length(t)
    tau(:,ii) = fcn_controller(t(ii),X(:,ii),p,traj);
    [Xd(:,ii),  Xd_dot(:,ii),  Xd_dotdot(:,ii) ] = fcn_trajectory(t(ii),traj);
    q = X(1:3,ii);
    dq = X(4:6,ii);
    Xe(:,ii) = fcn_p4(q,params);    %End-effector position
    Jc = fcn_J4(q,params);          %End-effector Jacobian
    Xe_dot(:,ii) = Jc*dq;                 %End-effector velocity
end


%% Plotting ----------------------------------------------------------------
%Plotting robot trajectories
figure(1)
clf

subplot(3,1,1)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,Xe,'-','LineWidth',1.5);
set(gca,'ColorOrderIndex',4)
h2 = plot(t,Xd,':','LineWidth',2);
hold off
ylabel('End effector pos. [m]')
legend([h1; h2], 'x','y','z','x_d','y_d','z_d')
grid on

subplot(3,1,2)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,Xe_dot,'-','LineWidth',1.5);
set(gca,'ColorOrderIndex',4)
h2 = plot(t,Xd_dot,':','LineWidth',2);
hold off
ylabel('Velocity dX_e/dt [m/s]')
%legend([h1; h2], 'v_x','v_y','v_z',...
%    'v_{dx}','v_{dy}','v_{dz}')
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
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,Xe-Xd,'LineWidth',2);
ylabel('Task-space error e(t) = X_d(t) - X_e(t) [m]')
xlabel('Time [s]')
legend(h1,'e_x','e_y','e_z')
grid on

figure(3)
clf
subplot(3,1,1)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,Xd,':','Linewidth',2);
legend(h1,'x_{d}','y_{d}','z_{d}')
ylabel('Desired positions [m]')
grid on
subplot(3,1,2)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
plot(t,Xd_dot,':','Linewidth',2)
ylabel('Desired velocities [m/s]')
grid on
subplot(3,1,3)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
plot(t,Xd_dotdot,':','Linewidth',2)
ylabel('Desired accelerations [m/s^2]')
grid on
xlabel('Time [s]')
pos = get(gcf,'position');
set(gcf,'position',[pos(1),pos(2)+pos(4)-600,560,600])

figure(4)
clf
plot([Xe(1,1) Xd(1,:)],[Xe(3,1) Xd(3,:)],':k','LineWidth',2)
hold on
plot(Xe(1,:),Xe(3,:),'k','LineWidth',1.5)
legend('X_d','X_e')
xlabel('X [m]')
ylabel('Z [m]')
grid on
axis equal

%Animating the robot
animateRobot(t,X,p,0)

