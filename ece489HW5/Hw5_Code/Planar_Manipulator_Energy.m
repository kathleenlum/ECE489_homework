function [E, K, P] = Planar_Manipulator_Energy(t, X)
% Starter code by Justin Yim 2023
% 
% INPUTS:
% t: (scalar) time in seconds
% X: n by 1 dynamical system state vector (coordinates and velocities)
%
% OUTPUTS:
% E: (scalar) total energy of the Planar_Manipulator
% K: (scalar) kinetic energy of the Planar_Manipulator
% P: (scalar) potentia energy of teh Planar_Manipulator

%%% copied from planar manipulator.m

global g L1 L2 m1 m2
%Current joint states
q = X(1:2,1);
q_dot = X(3:4,1);

%Forward kinematics, end-effector position
xe = L1*cos(q(1)) + L2*cos(q(1) + q(2));
ye = L1*sin(q(1)) + L2*sin(q(1) + q(2));
Xe = [xe; ye;];

%End-effector Jacobian (linear velocity)
Jv = [[-L1*sin(q(1))-L2*sin(q(1) + q(2)) -L2*sin(q(1) + q(2))];
      [ L1*cos(q(1))+L2*cos(q(1) + q(2))  L2*cos(q(1) + q(2))];];

%Inertia matrix (Modern robotics pg 275)
d11 = m1*L1^2 + m2*(L1^2 + L2^2 + 2*L1*L2*cos(q(2)));
d12 = m2*(L2^2 + L1*L2*cos(q(2)));
d21 = d12;
d22 = m2*L2^2;
M = [[d11 d12]
     [d21 d22]];

%Coriolis vector
c1 = -m2*L1*L2*sin(q(2))*(2*q_dot(1)*q_dot(2) + q_dot(2)^2);
c2 =  m2*L1*L2*q_dot(1)^2*sin(q(2));
C = [c1; c2];

%Gravity vector
g1 = (m1 + m2)*L1*g*cos(q(1)) + m2*g*L2*cos(q(1) + q(2));
g2 = m2*g*L2*cos(q(1) + q(2));
G = [g1; g2];

%%% copied from planar manipulator.m

K = 0.5 * q_dot' * M * q_dot;

% calculate potential energy

% going straight to use sine function because the gravity vector terms g1, g2 
% cosine is derivative of sine
P = (m1 + m2)*g*L1*sin(q(1)) + m2*g*L2*sin(q(1) + q(2));

% 4. Total Energy (E)
E = K + P;

% Replace the following expressions with your code
%E = 0;
%K = 0;
%P = 0;
