%% [copied from homework solutions] not kathleen original code

function p = CRS_FK(q)
% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% p: 3x1 column vector: end effector coordinates in frame 0 (all units of
% meters)
% INPUT:
% q: 3x1 column vector: joint coordinates (all units of radians)
% Your code goes here
theta1 = q(1, 1);
theta2 = q(2, 1);
theta3 = q(3, 1);
d1 = 0.254;
d2 = 0;
d3 = 0;
alpha1 = -pi/2;
alpha2 = 0;
alpha3 = 0;
a1 = 0;
a2 = 0.254;
a3 = 0.254;
% Defining the sub homogeneous transformation matricies using the DH
% parameters for frame 0 to frame 1
Rot_z1_theta1 = [cos(theta1), -sin(theta1), 0, 0;
sin(theta1), cos(theta1), 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;];
Trans_z1_d1 = [1, 0, 0, 0;
0, 1, 0, 0;
0, 0, 1, d1;
0, 0, 0, 1;];
Trans_x1_a1 = [1, 0, 0, a1;
0, 1, 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;];
Rot_z1_alpha1 = [1, 0, 0, 0;

0, cos(alpha1), -sin(alpha1), 0;
0, sin(alpha1), cos(alpha1), 0;
0, 0, 0, 1;];

H0_1 = (Rot_z1_theta1*Trans_z1_d1*Trans_x1_a1*Rot_z1_alpha1)
% Defining the sub homogeneous transformation matricies using the DH
% parameters for frame 1 to frame 2
Rot_z2_theta2 = [cos(theta2), -sin(theta2), 0, 0;
sin(theta2), cos(theta2), 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;];
Trans_z2_d2 = [1, 0, 0, 0;
0, 1, 0, 0;
0, 0, 1, d2;
0, 0, 0, 1;];
Trans_x2_a2 = [1, 0, 0, a2;
0, 1, 0, 0;

0, 0, 1, 0;
0, 0, 0, 1;];
Rot_z2_alpha2 = [1, 0, 0, 0;

0, cos(alpha2), -sin(alpha2), 0;
0, sin(alpha2), cos(alpha2), 0;
0, 0, 0, 1;];

H1_2 = (Rot_z2_theta2*Trans_z2_d2*Trans_x2_a2*Rot_z2_alpha2)
% Defining the sub homogeneous transformation matrices using the DH
% parameters for frame 2 to frame 3
Rot_z3_theta3 = [cos(theta3), -sin(theta3), 0, 0;
sin(theta3), cos(theta3), 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;];
Trans_z3_d3 = [1, 0, 0, 0;
0, 1, 0, 0;
0, 0, 1, d3;
0, 0, 0, 1;];
Trans_x3_a3 = [1, 0, 0, a3;
0, 1, 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;];
Rot_z3_alpha3 = [1, 0, 0, 0;

0, cos(alpha3), -sin(alpha3), 0;
0, sin(alpha3), cos(alpha3), 0;
0, 0, 0, 1;];

H2_3 = (Rot_z3_theta3*Trans_z3_d3*Trans_x3_a3*Rot_z3_alpha3);
% Compute the overall transformation matrix from frame 0 to frame 2
H0_2 = (H0_1 * H1_2);
% Compute the overall transformation matrix from frame 0 to frame 2
H0_3 = (H0_1 * H1_2 * H2_3);
P0 = H0_3*[0; 0; 0; 1]; % End-effector position w.r.t world frame
p = P0(1:end-1, 1);
end