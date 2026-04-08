% SE422/ME446/ECE489 Homework 4, Prof. Justin Yim 2025

%% Symbolic derivation ----------------------------------------------------
% Define symbolic variables ---------------------
q = sym('theta',[3,1]); % Joint angles (radians)


syms m1 m2 m3 g;
g0 = [0 0 -g];
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3% Inertia tensor params
masses = [m1, m2, m3];

%Inertia tensors (these are diagonal, as given by HW4 problem 1
I1 = diag([Ixx1 Iyy1 Izz1]);
I2 = diag([Ixx2 Iyy2 Izz2]);
I3 = diag([Ixx3 Iyy3 Izz3]);

I  = {I1 I2 I3};

% Compute kinematics and dynamics ---------------
% Kinematics

% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% latex code for forwards kinematics to express the center of mass location
% of each link in the base frame 0

% take code from HW2 plotCRS.m
%define values for DH parameters as given in HW

%use symbols for variables
syms theta1 theta2 theta3;
syms L1 L2 L3;
syms theta1dot theta2dot theta3dot;
syms c1 c2 c3; % position of center of mass of each link

q_dot = [theta1dot; theta2dot; theta3dot];

a1 = 0;
alpha1 = -(sym(pi))/2;
d1 = L1;%meters 


a2 = L2;%meters 
alpha2 = 0;
d2 = 0;


a3 = L3;%meters 
alpha3 = 0;
d3 = 0; 


% Compute the transformation matrix for the first link
H0_1 = [[cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)];
         [sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)];
         [0 sin(alpha1) cos(alpha1) d1];
         [0 0 0 1];];

R0_1 = H0_1(1:3, 1:3); % Rotation of Frame 1 relative to Frame 0

% use transformation matrix in the world frame and project it onto the
% center of mass of the first link
r1_c1 = [0;c1;0;1];
f = simplify(H0_1*r1_c1);
comfirstlink = f(1:3);

% Compute the transformation matrix for the second link
H1_2 = [[cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)];
          [sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)];
          [0 sin(alpha2) cos(alpha2) d2];
          [0 0 0 1];];

H0_2 = H0_1 * H1_2;
R0_2 = H0_2(1:3, 1:3); % Rotation of Frame 2 relative to Frame 0

% use transformation matrix in the world frame and project it onto the
% center of mass of the second link
r2_c2 = [-c2;0;0;1];
s = simplify(H0_2*r2_c2);
comsecondlink = s(1:3);


% Compute the transformation matrix for the third link
H2_3 = [[cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)];
          [sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)];
          [0 sin(alpha3) cos(alpha3) d3];
          [0 0 0 1];];

H0_3 = H0_2 * H2_3;
R0_3 = H0_3(1:3, 1:3); % Rotation of Frame 3 (End-Effector) relative to Frame 0
o0_3 = H0_3;

R0_i = {R0_1 R0_2 R0_3};

% use transformation matrix in the world frame and project it onto the
% center of mass of the third link
r3_c3 = [-c3; 0;0;1];
t = simplify(H0_3*r3_c3);
comthirdlink = t(1:3);

com_links = [comfirstlink, comsecondlink, comthirdlink];

% Velocity kinematics (Jacobians)
%Positions of center of mass (CoM) of each link
%Position of entire system CoM
CoM = (comfirstlink*m1 + comsecondlink*m2 + comthirdlink*m3)/(m1 + m2 + m3);

%Jacobian for each CoM linear velocity
Jv1 = simplify(jacobian(comfirstlink(1:3),q));
Jv2 = simplify(jacobian(comsecondlink(1:3),q));
Jv3 = simplify(jacobian(comthirdlink(1:3),q));
%End-effector position Jacobian
Jve = simplify(jacobian(o0_3(1:3),q));

Jv = {Jv1 Jv2 Jv3 Jve};

%angular velocities of each link uses the angular velocity terms of the

% compute angular velocity of each joint
z0 = [0;0;1];
z1 = simplify(R0_1*[0;0;1]); 
z2 = simplify(R0_2 * [0;0;1]);

Jw1 = [z0,[0;0;0], [0;0;0]];
Jw2 = [z0, z1, [0;0;0]];
Jw3 = [z0, z1, z2]; % Angular component for the third link
Jw = {Jw1, Jw2, Jw3}; % Combine angular components of the Jacobian


% Potential energy and G vector
% HW4 q1a
function pot_e = findGvector(masses, g0, com_links)
    pot_e = 0;
    for i = 1:3
       pot_e = pot_e +  (-masses(i)*g0*com_links(:, i));
    end
end
pot_e = findGvector(masses, g0, com_links);
% G matrix!!!
output_pot_e = latex(simplify(pot_e))

% HW4 q1b

%G = jacobian(pot_e,q).';
G = [diff(pot_e,q(1)); diff(pot_e,q(2)); diff(pot_e,q(3))];
output_G2 = latex(simplify(G))

% Kinetic energy and M matrix
function M = massMatrix(masses, Jv, R0_i, I, Jw)
    M = zeros(3,3);
    
    M = masses(1)*transpose(Jv{1})*Jv{1} + transpose(Jw{1})*R0_i{1}*I{1}*transpose(R0_i{1})*Jw{1} + ...
       masses(2)*transpose(Jv{2})*Jv{2} + transpose(Jw{2})*R0_i{2}*I{2}*transpose(R0_i{2})*Jw{2} + ...
       masses(3)*transpose(Jv{3})*Jv{3} + transpose(Jw{3})*R0_i{3}*I{3}*transpose(R0_i{3})*Jw{3};
    
    M = simplify(M);
end
M = massMatrix(masses, Jv, R0_i, I, Jw);
output_M22 = latex(M(2,2))
output_M33 = latex(M(3,3))

kin_e = simplify(0.5*transpose(q_dot)*M*q_dot);

output_kin_e = latex(kin_e)

% Christoffel symbols (C matrix)
function C = christoffelSymbols(M, q, q_dot)
    C = vpa(zeros(3,3));
    for k=1:3
        for j=1:3
            for i=1:3
                C(k,j) = C(k,j)+ 0.5*(diff(M(k,j), q(i)) + ...
                                      diff(M(k,i), q(j)) - ...
                                      diff(M(i,j), q(k))) *q_dot(i);
            end 
        end
    end
end
C = christoffelSymbols(M, q, q_dot);
output_C = latex(simplify(C))

%% Numerical evaluation ---------------------------------------------------
% Define numeric quantities
g_num = 9.81; % m/s^2
L_num = 0.254; %meters
c_num = 0.127; % meters
m1_num = 3.12; %kg
m2_num = 1.30; %kg
m3_num = 1.14; %kg

I1_num = [[0.0155 0 0];
          [0 0.0096 0];
          [0 0 0.0150]];

I2_num = [[0.0021 0 0];
          [0 0.0144 0];
          [0 0 0.0104]];

I3_num = [[0.0010 0 0];
          [0 0.0111 0];
          [0 0 0.0106]];

q_num = [0; -pi/2; 0];
q_dot_num= [0;0;0];
tau = [1;0;0];

% Evaluate forward dynamics using "subs" function
function qdotdot = forwardDynamicsEOM(q_num, q_dot_num, tau, M, C, G, q, q_dot)
    M_num = subs(M, q, q_num);
    M_num = subs(M_num, q_dot, q_dot_num);
    C_num = subs(C, q, q_num);
    C_num = subs(C_num, q_dot, q_dot_num);
    
    G_num = subs(G, q, q_num);
    G_num = subs(G_num, q_dot, q_dot_num);

    
    %COMBINE SUBSTITUTED VALUES INTO MANIPULATOR EoM FROM LECTURE NOTES
    qdotdot = M_num\(tau - (C_num * q_dot_num) - G_num);
    
end

part2a = forwardDynamicsEOM(q_num, q_dot_num, tau, M, C, G, q, q_dot);
output_part2a = latex(simplify(part2a))

% Evaluate G(q) for zero configuration q = [0;0;0] and stationary 
% torques required to hold the robot in this state is just G(q)
G_stationary = subs(G, q, [0;0;0]);
part2b = latex(subs(G_stationary, q_dot, [0;0;0]))

% Evaluate inverse dynamic using "subs" function


% Now evaluate at the specific state for 2c
q_test = [0; -pi/4; pi/3];
dq_test = [1; 1; 1];
tau_test = [-3; -2; 1];

part2c= forwardDynamicsEOM(q_test, dq_test, tau_test, M, C, G, q, q_dot);
% Create a parameter structure for substitution
params = {L1, L2, L3, c1, c2, c3, m1, m2, m3, g, ...
          Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3, pi};
values = {L_num, L_num, L_num, c_num, c_num, c_num, m1_num, m2_num, m3_num, g_num, ...
          I1_num(1,1), I1_num(2,2), I1_num(3,3), ...
          I2_num(1,1), I2_num(2,2), I2_num(3,3), ...
          I3_num(1,1), I3_num(2,2), I3_num(3,3), pi};


aaapart2c = subs(part2c, params, values);
output_part2c = double(aaapart2c)


q_num = [0; -pi/4;pi/3];
q_dot_num= [1;1;1];
q_dotdot = [10;1;-1];

M_num = subs(M, q, q_num);
M_num = subs(M_num, q_dot, q_dot_num);
C_num = subs(C, q, q_num);
C_num = subs(C_num, q_dot, q_dot_num);

G_num = subs(G, q, q_num);
G_num = subs(G_num, q_dot, q_dot_num);


%COMBINE SUBSTITUTED VALUES INTO MANIPULATOR EoM FROM LECTURE NOTES
tau_part2d = M_num*q_dotdot + C_num * q_dot_num + G_num;

taupart2d = subs(tau_part2d , params, values);
output_part2d = double(taupart2d)