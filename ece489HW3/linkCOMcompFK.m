% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% latex code for forwards kinematics to express the center of mass location
% of each link in the base frame 0

% take code from HW2 plotCRS.m
%define values for DH parameters as given in HW

%use symbols for variables
syms theta1 theta2 theta3;
syms L;
syms theta1dot theta2dot theta3dot;
a1 = 0;
alpha1 = -(sym(pi))/2;
d1 = L;%meters 


a2 = L;%meters 
alpha2 = 0;
d2 = 0;


a3 = L;%meters 
alpha3 = 0;
d3 = 0; 


% Compute the transformation matrix for the first link
link1 = [[cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)];
         [sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)];
         [0 sin(alpha1) cos(alpha1) d1];
         [0 0 0 1];];

H0_1 = link1;
% use transformation matrix in the world frame and project it onto the
% center of mass of the first link
r1_c1 = [0;sym('c1');0;1];
f = simplify(H0_1*r1_c1);
firstlink = f(1:3);

% Compute the transformation matrix for the second link
link2 = [[cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)];
          [sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)];
          [0 sin(alpha2) cos(alpha2) d2];
          [0 0 0 1];];

H0_2 = H0_1 * link2;
% use transformation matrix in the world frame and project it onto the
% center of mass of the second link
r2_c2 = [-sym('c2');0;0;1];
s = simplify(H0_2*r2_c2);
secondlink = s(1:3);


% Compute the transformation matrix for the third link
link3 = [[cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)];
          [sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)];
          [0 sin(alpha3) cos(alpha3) d3];
          [0 0 0 1];];

H0_3 = H0_2 * link3;
% use transformation matrix in the world frame and project it onto the
% center of mass of the third link
r3_c3 = [-sym('c3'); 0;0;1];
t = simplify(H0_3*r3_c3);
thirdlink = t(1:3);

%angular velocities of each link uses the angular velocitiy terms of the
%joint variables

R0_1 = H0_1(1:3, 1:3); % Rotation of Frame 1 relative to Frame 0

R0_2 = H0_2(1:3, 1:3); % Rotation of Frame 2 relative to Frame 0

R0_3 = H0_3(1:3, 1:3); % Rotation of Frame 3 (End-Effector) relative to Frame 0

% compute angular velocity of each joint
Jw1 = [0;0;1];
Jw2 = simplify(Jw1+ R0_1*[0;0;1]);
Jw3 = simplify(Jw2+R0_2 * [0;0;1]); % Angular component for the third link
Jw = [Jw1, Jw2, Jw3]; % Combine angular components of the Jacobian

q_dot = [theta1dot; theta2dot; theta3dot];

%compute the expression for angular velocity of each link using the jacobian
%to map between end effector velocity and joint velocity
J1 = [Jw1 [0;0;0] [0;0;0]]; % effect of link1
output = simplify(J1 * q_dot);
latex(output(1:3))

J2 = [Jw1 Jw2 [0;0;0]];%effect of link2
output2 = simplify(J2 * q_dot);
latex(output2(1:3))

J3 = Jw;%effect of link3
output3 = simplify(J3 * q_dot);
latex(output3(1:3))

