function p = DH_calc(q)
% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% p: 3x1 column vector: end effector coordinates in frame 0 (all units of
% meters)
% INPUT:
% q: 3x1 column vector: joint coordinates (all units of radians)

% Your code goes here

a1 = 0;
alpha1 = -sym(pi)/2;
d1 = 0.254;%meters 
theta1 = sym("theta1");%q(1); 

a2 = 0.254;%meters 
alpha2 = 0;
d2 = 0;
theta2 = sym("theta2");%q(2); 

a3 = 0.254;%meters 
alpha3 = 0;
d3 = 0; 
theta3 = sym("theta3");%q(3);

% Compute the transformation matrix for the first link
link1 = [[cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)];
         [sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)];
         [0 sin(alpha1) cos(alpha1) d1];
         [0 0 0 1];];

fprintf('link1 =\n');
disp(link1)

H0_1 = simplify(link1);
% Compute the transformation matrix for the second link
link2 = [[cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)];
          [sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)];
          [0 sin(alpha2) cos(alpha2) d2];
          [0 0 0 1];];

H0_2 = simplify(H0_1 * link2);

fprintf('link2 =\n');
disp(link2)

% Compute the transformation matrix for the third link
link3 = [[cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)];
          [sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)];
          [0 sin(alpha3) cos(alpha3) d3];
          [0 0 0 1];];

fprintf('link3 =\n');
disp(link3)

H0_3 = simplify(H0_2 * link3);

fprintf('end affector =\n');
disp(H0_3)

O0 = [0;0;0;1];
O0_1 = H0_1*[0; 0; 0; 1]; % frame 1 origin w.r.t. world frame
O0_2 = H0_2*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame
O0_3 = H0_3*[0; 0; 0; 1]; % frame 3 origin w.r.t world frame
p = H0_3*[0;0;0;1]; % Extract the end effector position

%p = [0;0;0]; % replace this

pts = [O0,O0_1,O0_2,O0_3,p];

fprintf('H_0_1 =\n');
disp(latex(H0_1))
fprintf('H_0_2 =\n');
disp(latex(H0_2))
fprintf('H_0_3 =\n');
disp(latex(H0_3))

end