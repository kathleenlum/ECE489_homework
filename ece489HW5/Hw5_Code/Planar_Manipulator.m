function X_dot = Planar_Manipulator(t,X)
% Written by Joao Ramos
% Modified by Justin Yim 2023

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

%External force applied to end-effector
Fe = ExternalForce(t);

%Define control law
tau = [0; 0]; 

%Compute dynamics
q_dotdot = M\(tau - C - G + Jv'*Fe);

X_dot = [q_dot; q_dotdot];

end