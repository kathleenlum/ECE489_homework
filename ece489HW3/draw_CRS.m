function p = draw_CRS(q, q_dot)

% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% p: 3x1 column vector: end effector coordinates in frame 0 (all units of
% meters)
% INPUT:
% q: 3x1 column vector: joint coordinates (all units of radians)
% q_dot: 3x1 joint velocities (units of rad/s)

% take code from HW2 plotCRS.m

a1 = 0;
alpha1 = -pi/2;
d1 = 0.254;%meters 
theta1 = q(1); 

a2 = 0.254;%meters 
alpha2 = 0;
d2 = 0;
theta2 = q(2); 

a3 = 0.254;%meters 
alpha3 = 0;
d3 = 0; 
theta3 = q(3);

% Compute the transformation matrix for the first link
link1 = [[cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)];
         [sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)];
         [0 sin(alpha1) cos(alpha1) d1];
         [0 0 0 1];];

%fprintf('link1 =\n');
%disp(link1)

H0_1 = link1;
% Compute the transformation matrix for the second link
link2 = [[cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)];
          [sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)];
          [0 sin(alpha2) cos(alpha2) d2];
          [0 0 0 1];];

H0_2 = H0_1 * link2;

%fprintf('link2 =\n');
%disp(link2)

% Compute the transformation matrix for the third link
link3 = [[cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)];
          [sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)];
          [0 sin(alpha3) cos(alpha3) d3];
          [0 0 0 1];];

%fprintf('link3 =\n');
%disp(link3)

H0_3 = H0_2 * link3;

%fprintf('end affector =\n');
%disp(H0_3)


O0 = [0;0;0;1];
O0_1 = H0_1*[0; 0; 0; 1]; % frame 1 origin w.r.t. world frame
O0_2 = H0_2*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame
O0_3 = H0_3*[0; 0; 0; 1]; % frame 3 origin w.r.t world frame
p = H0_3*[0;0;0;1]; % Extract the end effector position

fprintf('p =\n');
disp(p(1:3))

%find the linear component of jacobian for first, second and third links

% Linear Jacobian columns: Ji = zi-1 x (p - Oi-1)
J_lin = computeJacobian(q,0.254);

J = [J_lin(:,1), J_lin(:,2), J_lin(:,3)]; % Resulting 3x3 matrix
fprintf('J =\n');
disp(J);

% find the angular component of the Jacobian for end-effector angular velocity
%{
R0_1 = H0_1(1:3, 1:3); % Rotation of Frame 1 relative to Frame 0

R0_2 = H0_2(1:3, 1:3); % Rotation of Frame 2 relative to Frame 0

R0_3 = H0_3(1:3, 1:3); % Rotation of Frame 3 (End-Effector) relative to Frame 0

Jw1 = [0;0;1];
Jw2 = R0_1*[0;0;1];
Jw3 = R0_2 * [0;0;1]; % Angular component for the third link
Jw = [Jw1, Jw2, Jw3]; % Combine angular components of the Jacobian
J = [linear_J; Jw]; % Combine linear and angular components of the Jacobian
%}

%calculate end-effector velocity
v = J * q_dot;
fprintf('v =\n');
disp(v);


pts = [O0,O0_1,O0_2,O0_3,p];
%% Plotting
close all;
% Draw the robot stick figure
plot3(pts(1,:),pts(2,:),pts(3,:),'.-k','LineWidth',3,'MarkerSize',20)
% plot3 draws a sequence of line segments connecting points in 3D space;
% the .-k option says to use a dot at each point (.), draw a solid line
% between subsequent points (-), and use black for the color (k). See the
% online MATLAB documentation for more details (it's very convenient).

hold on % allow more objects to be drawn without erasing earlier objects

% Label the point p
offset = 0.08; %displacement of text
text(p(1)+offset,p(2)+offset,p(3)+offset,'p')
% text writes text at a specified point.

% Length for frame axis vectors
scale = 0.5; 

%Plotting frame 0 magenta
quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[scale;0;0],[0;scale;0],[0;0;scale],'m')
% quiver3 draws a set of arrows in 3D space
v_scale = 0.5;
quiver3(p(1), p(2), p(3), v(1), v(2), v(3), v_scale, 'Color', 'r', 'LineWidth', 2, 'AutoScale', 'off');

% Plot individual velocity components emanating off of p
quiver3(p(1), p(2), p(3), J(1,1), J(2,1), J(3,1), v_scale, 'Color','b', 'LineWidth', 2, 'AutoScale','off');
quiver3(p(1), p(2), p(3), J(1,2), J(2,2), J(3,2), v_scale, 'Color','b', 'LineWidth', 2, 'AutoScale','off');
quiver3(p(1), p(2), p(3), J(1,3), J(2,3), J(3,3), v_scale, 'Color','b', 'LineWidth', 2, 'AutoScale','off');

% Configure the axes
axis equal % Important! Scales all three axis the same so the robot isn't squished and stretched
grid on % Draw a light grid in the background of the figure axes
set(gca,'xlim',[-1,1],'ylim',[-1,1],'zlim',[0,2]) % adjust the plot space bounds
xlabel('X_0 (m)')
ylabel('Y_0 (m)')
zlabel('Z_0 (m)')


end