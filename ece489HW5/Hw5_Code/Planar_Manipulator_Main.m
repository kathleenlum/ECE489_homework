% Planar_Manipulator_Main.m
% Written by Joao Ramos
% Modified by Justin Yim 2023

clc;
global g L1 L2 m1 m2 
g = 9.81;
L1 = 1;
m1 = 2;
L2 = L1;
m2 = m1/2;

q0 = [0; 0];  %Initial angle
q_dot0 = [1; 3];   %Initial angular velocity
T = 8;             %Simulation time

DeltaT = 1/1000;  % (seconds) Time step for the simulation
%% ------------------------------------------------------------------------
% Simulating the planar manipulator
X0 = [q0; q_dot0;];
tic
[time, X] = Euler_Integrator(@Planar_Manipulator, [0 T], X0, DeltaT);%ode45(@Planar_Manipulator,[0 T],X0);
toc

%% ------------------------------------------------------------------------
% Plotting

%Plotting solutions
figure(1);
subplot(2,1,1),plot(time,X(:,1:2))
grid on
xlabel('Time [s]')
ylabel('Joint angles [rad]')
legend('q_1','q_2')
subplot(2,1,2),plot(time,X(:,3:4))
grid on
xlabel('Time [s]')
ylabel('Joint angular velocity [rad/s]')
legend('dq_1/dt','dq_2/dt')

% Plotting energy
T_steps = numel(time);
E = zeros(T_steps, 1);
K = zeros(T_steps, 1);
P = zeros(T_steps, 1);
for ii = 1:T_steps
    [E(ii), K(ii), P(ii)] = Planar_Manipulator_Energy(time(ii), X(ii,:).');
end

figure(2);
clf
plot(time, E, 'k')
hold on
plot(time, K, 'r')
plot(time, P, 'b')
xlabel('Time (s)')
ylabel('Energy (J)')
grid on
legend('total E','KE','PE')

%% ------------------------------------------------------------------------
% Animating
Animate_Planar_Manipulator(time, X, 0);
% set last input to 1 if you would like to record video file and 0 if not

