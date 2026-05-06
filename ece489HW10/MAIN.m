% 3-link manipulator simulation
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023

addpath gen
addpath fcns

% --- parameters ---
% Get the parameters about the robot and simulation settings
p = get_params();
params = p.params;
%Actuators reflected inertia matrix:
N = 25;
Ir = 1.4e-5;
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir]];

%Initial condition:
q0 = [0; -2.63; 2.13];       
dq0 = [0; 0; 0];
ic = [q0; dq0;]; 
% figure
% plotRobot(q0,p,[0 0.5 1]);

%Waypoints
P_des = [0,    0,   0,    0.35,  0.35,  0,    0;
         0.4,  0.4, 0.4,  0.15, -0.15, -0.4, -0.4;
         0.35, 0.1, 0.05, 0.15,  0.15,  0.4,  0.3];
thresh = 0.005;
F_des = [0;0;10];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up trajectory variables here (if you want)---------------------------

% Your code can go here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
%Simulate unconstrained dynamics
tfinal = 12;
[t,X] = ode45(@(t,X)dyn_manip(t,X,p),[0 tfinal],ic);
X = X'; 

%Reconstructing control inputs and desired trajectory for plotting
tau = zeros(3,length(t));
P_w = zeros(3,length(t));
P_ee = zeros(3,length(t));
F_ext = zeros(3,length(t));
for ii = 1:length(t)
    tau(:,ii) = fcn_controller(t(ii),X(:,ii),p);
    P_w(:,ii) = fcn_p4(X(:,ii),params);
    [F_ext(:,ii), P_ee(:,ii)] = fcn_ExternalForce(t(ii),X(:,ii),p);
end

%% Plotting
figure(1)
clf

subplot(2,1,1)
plot(0,0,'-k')
hold on
set(gca,'ColorOrderIndex',4)
h1 = plot(t,P_ee,'-','LineWidth',1.5);
hold off
ylabel('End effector pos. [m]')
legend(h1, 'x','y','z')
grid on

subplot(2,1,2)
plot(t,tau,'LineWidth',1.5)
ylabel('Joint torque \tau [Nm]')
xlabel('Time [s]')
legend('\tau_1','\tau_2','\tau_3')
grid on
pos = get(gcf,'position');
set(gcf,'position',[pos(1),pos(2)+pos(4)-600,560,600])

figure(2)
clf
plot3(P_ee(1,:),P_ee(2,:),P_ee(3,:),'k','LineWidth',1.5)
hold on
plot3(P_des(1,:),P_des(2,:),P_des(3,:),'or')
for ii = 1:size(P_des,2)
    text(P_des(1,ii),P_des(2,ii),P_des(3,ii)+0.02,num2str(ii))
end
title('End effector path')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
grid on
axis equal

figure(3)
clf
subplot(3,1,1),plot(t,F_ext(1,:),'k','LineWidth',1.5)
set(gca,'ylim',[-25,25])
title('Contact forces')
ylabel('F_x [N]')
grid on
subplot(3,1,2),plot(t,F_ext(2,:),'k','LineWidth',1.5)
set(gca,'ylim',[-25,25])
ylabel('F_y [N]')
grid on
subplot(3,1,3),plot(t,F_ext(3,:),'k','LineWidth',1.5)
set(gca,'ylim',[-25,25])
xlabel('time [s]')
ylabel('F_z [N]')
grid on


dist = zeros(size(P_ee,2),size(P_des,2),3);
for ii = 1:3
    dist(:,:,ii) = P_ee(ii,:).'-P_des(ii,:);
end
reached = zeros(1,size(P_des,2));
for ii = 1:size(P_des,2)
    if any(reached(1:(ii-1)))
        last_reached = find(reached,1,'last');
        if any(all(abs(dist(last_reached:end,ii,:))<thresh,3))
            reached(ii) = find(all(abs(dist(last_reached:end,ii,:))<thresh,3),1,'first');
        end
    else
        if any(all(abs(dist(:,ii,:))<thresh,3))
            reached(ii) = find(all(abs(dist(:,ii,:))<thresh,3),1,'first');
        end
    end
end
disp(['Reached ',num2str(sum(reached>0)),' out of ',num2str(size(P_des,2)),' waypoints'])
disp(['Peak x, y, or z force ',num2str(max(abs(F_ext(:)))), ' N out of 20 N'])
if reached(end)
    atButton = all(abs(dist(:,ii,:))<thresh,3);
    [err, F_ind] = min(abs(F_ext(3,:)-F_des(3))+1/atButton);
    disp(['Final force [',num2str(F_ext(:,F_ind).'),'] N (target [0, 0, 10] N)'])
else
    disp('Waypoint 7 not reached -- reach waypoint 7 to receive a force reading')
end





figure(4)
clf
subplot(3,1,1)
plot(t,P_ee(1,:),'k-','LineWidth',1.5);
hold on
grid on
ylabel('X [m]')
title('Reaching waypoints')
subplot(3,1,2)
plot(t,P_ee(2,:),'k-','LineWidth',1.5);
hold on
grid on
ylabel('Y [m]')
subplot(3,1,3)
plot(t,P_ee(3,:),'k-','LineWidth',1.5);
hold on
grid on
ylabel('Z [m]')
xlabel('T [s]')

for ii = 1:size(P_des,2)
    if reached(ii)
        t_r = t(reached(ii));
        for jj = 3:-1:1
            subplot(3,1,jj)
            plot([t_r,t_r],P_des(jj,ii)+[-1,1]*thresh,'.-g',...
                'linewidth',1.5,'markersize',10);
        end
        text(t_r,P_des(jj,ii)+0.05,num2str(ii))
    end
end

%Animating the robot:
animateRobot(t,X,p,1)
