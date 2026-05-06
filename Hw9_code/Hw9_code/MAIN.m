% 3-link manipulator simulation
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023

addpath gen
addpath fcns

% parameters --------------------------------------------------------------
% Get the parameters about the robot and simulation settings
p = get_params();
params = p.params;

%Initial conditions:
q0 = [0; -2.63; 2.13];%[0;-1.4;1.7];%[0;-0.8108;1.6215];%[0; -2.63; 2.13];
dq0 =[0; 0; 0]; %[0;1;-1];%[0; 0; 0];
ic = [q0; dq0];

%Plot the robot at initial conditions:
% figure
% plotRobot(ic,p,[0 0.5 1]);

% Simulation --------------------------------------------------------------
%--------------------------------------------------------------------------
%Simulate unconstrained dynamics
tfinal = 5;
options = odeset('Events',@wall_contact); 
[t_uncons,X_uncons] = ode45(@(t,X)UNCONS_dyn_manip(t,X,p),[0 tfinal],ic, options);
X_uncons = X_uncons'; 

%--------------------------------------------------------------------------
%Calculate impact map
[X, Fwall_imp] = impact_reset(X_uncons(:,end));

%--------------------------------------------------------------------------
%Simulate constrained dynamics after impact
if (t_uncons(end) < tfinal)   %Impact occured
    [t_cons,X_cons] = ode45(@(t,X)CONS_dyn_manip(t,X,p),[t_uncons(end)+1e-6 tfinal],X);
    X_cons = X_cons';
else                          %Impact did not occur
    t_cons = [];
    X_cons = [];
end
%--------------------------------------------------------------------------

%Concatenating both dynamics
t = [t_uncons; t_cons];
X = [X_uncons X_cons;];

%Reconstructing control inputs and desired trajectory for plotting
tau = zeros(3,length(t));
X_ee = zeros(3,length(t));
X_ee_dot = zeros(3,length(t));
Xd = zeros(3,length(t));
Xd_dot = zeros(3,length(t));
Xd_dotdot = zeros(3,length(t));
F_c = zeros(3,length(t));
Fwall = zeros(1,length(t_cons));
for ii = 1:length(t)
    tau(:,ii) = fcn_controller(t(ii),X(:,ii),p,t(ii)>t_uncons(end));
    X_ee(:,ii) = fcn_p4(X(:,ii),params);
    q = X(1:3,ii);
    dq = X(4:6,ii);
    Je = fcn_J4(q,params);          %End-effector Jacobian
    X_ee_dot(:,ii) = Je*dq;         %End-effector velocity
    F_c(:,ii) = fcn_ExternalForce(t(ii),X(:,ii),p);
    [Xd(:,ii),  Xd_dot(:,ii),  Xd_dotdot(:,ii) ] = fcn_trajectory(t(ii));
end
for ii = 1:length(t_cons)
    [~, Fwall(:,ii)] = CONS_dyn_manip(t_cons(ii),X_cons(:,ii),p);
end

%% Plotting ----------------------------------------------------------------
%Plotting robot trajectories
figure(1)
clf
subplot(3,1,1)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,X_ee,'-','LineWidth',1.5);
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
h1 = plot(t,X_ee_dot,'-','LineWidth',1.5);
set(gca,'ColorOrderIndex',4)
h2 = plot(t,Xd_dot,':','LineWidth',2);
hold off
ylabel('Velocity dX_e/dt [m/s]')
legend([h1; h2], 'v_x','v_y','v_z',...
    'v_{dx}','v_{dy}','v_{dz}')
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
set(gca,'ColorOrderIndex',5)
h1 = plot(t,X_ee(2:3,:)-Xd(2:3,:),'LineWidth',2);
ylabel('Task-space error e(t) = X_d(t) - X_e(t) [m]')
xlabel('Time [s]')
legend(h1,'e_y','e_z')
grid on

figure(3)
clf
h1 = plot(t,0.35-X_ee(1,:),'k','LineWidth',2);
ylabel('Wall distance [m]')
xlabel('Time [s]')
grid on

figure(4)
clf
plot3(X_ee(1,:),X_ee(2,:),X_ee(3,:),'k','LineWidth',2)
hold on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
grid on
axis equal

figure(5)
clf
plot(t_cons,Fwall,'k','LineWidth',2)

hold on
plot(t,F_c(1,:),'m','Linewidth',2)
xlabel('Time [s]')
ylabel('Contact Force F_c [N]')
legend('Fwall','F_c')
grid on

%Animating the robot
animateRobot(t,X,p,0)
