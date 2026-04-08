function plotThreeLinkHW1(q, params)
% This function plots a three-link robot arm for homework 1.
%
% INPUTS:
% q is a 3x1 vector of the joint angles [theta_1; theta_2; theta_3]
% params is a 3x1 vector of the link lengths [L_1; L_2; L_3]
theta_1 = q(1); %map theta1 to the first angle in q
theta_2 = q(2); %map theta2 to the second angle in q
theta_3 = q(3); %map theta3 to the third angle in q
L_1 = params(1); %map length1 to first length in params
L_2 = params(2); %map length1 to second length in params
L_3 = params(3); %map length1 to third length in params
%% Geometry and Transformations
%First joint kinematics -- rotation about the z axis
R0_1 = [[cos(theta_1) -sin(theta_1) 0]; % 1st joint rotation matrix: rotation of frame 1 with respect to frame 0
      [sin(theta_1) cos(theta_1) 0];
      [0 0 1];];
d0_1 = [0; 0; 0]; % displacement of frame 1 w.r.t. frame 0
H0_1 = [[R0_1 d0_1]; % homogeneous transformation matrix of frame 1 wr.t. frame 0
      [0 0 0 1];];
%Second joint kinematics -- rotation about the x axis
R1_2 = [[1 0 0]; % 2nd joint rotation matrix: rotation of frame 2 with respect to frame 1
      [0 cos(theta_2) -sin(theta_2)];
      [0 sin(theta_2) cos(theta_2)];];
d1_2 = [0; 0; L_1]; % displacement of frame 2 w.r.t. frame 1
H1_2 = [[R1_2 d1_2]; % homogeneous transformation matrix of frame 2 wr.t. frame 1
      [0 0 0 1];];
%Third joint kinematics -- rotation about the x axis
R2_3 = [[1 0 0]; % 3rd joint rotation matrix: rotation of frame 3 with respect to frame 2
      [0 cos(theta_3) -sin(theta_3)];
      [0 sin(theta_3) cos(theta_3)];];
d2_3 = [0; L_2; 0]; % displacement of frame 3 w.r.t. frame 2
H2_3 = [[R2_3 d2_3]; % homogeneous transformation matrix of frame 3 wr.t. frame 2
      [0 0 0 1];];
% Do your math here
% Compute the overall transformation matrix from the base to the end effector
H0_2 = H0_1 * H1_2; % Transformation from frame 0 to frame 2
H0_3 = H0_2 * H2_3; % Transformation from frame 0 to frame 3
% Points in the world frame (frame 0)
O0_1 = H0_1*[0; 0; 0; 1]; % frame 1 origin w.r.t. world frame
O0_2 = H0_2*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame
O0_3 = H0_3*[0; 0; 0; 1]; % frame 3 origin w.r.t. world frame
P0 = H0_3*[0; L_3; 0; 1]; % End-effector position w.r.t world frame
% Points to draw (frame 1 origin, frame 2 origin, frame 3 origin, end-effector)
pts = [O0_1, O0_2, O0_3, P0];
%% Plotting -- pretty much copied from HW1_Example.m
figure(1) % Open figure window 1
clf % Clear figure window 1 (erase everything in it)
% Draw the robot arm using the points calculated
plot3(pts(1,:), pts(2,:), pts(3,:), '-o', 'MarkerSize', 10, 'LineWidth', 2);
hold on % allow more objects to be drawn without erasing earlier objects
% Label the point p
offset = 0.08; %displacement of text
text(P0(1)+offset,P0(2)+offset,P0(3)+offset,'p')
% text writes text at a specified point.
% Length for frame axis vectors
scale = .25;
%Plotting frame 0 green
quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[scale;0;0],[0;scale;0],[0;0;scale],'g')
% quiver3 draws a set of arrows in 3D space
%Plotting frame 1 blue
quiver3(H0_1(1,4),H0_1(2,4),H0_1(3,4),scale*H0_1(1,1),scale*H0_1(2,1),scale*H0_1(3,1),'b')
quiver3(H0_1(1,4),H0_1(2,4),H0_1(3,4),scale*H0_1(1,2),scale*H0_1(2,2),scale*H0_1(3,2),'b')
quiver3(H0_1(1,4),H0_1(2,4),H0_1(3,4),scale*H0_1(1,3),scale*H0_1(2,3),scale*H0_1(3,3),'b')
%Plotting frame 2 magenta
quiver3(H0_2(1,4),H0_2(2,4),H0_2(3,4),scale*H0_2(1,1),scale*H0_2(2,1),scale*H0_2(3,1),'m')
quiver3(H0_2(1,4),H0_2(2,4),H0_2(3,4),scale*H0_2(1,2),scale*H0_2(2,2),scale*H0_2(3,2),'m')
quiver3(H0_2(1,4),H0_2(2,4),H0_2(3,4),scale*H0_2(1,3),scale*H0_2(2,3),scale*H0_2(3,3),'m')
%Plotting frame 3 cyan
quiver3(H0_3(1,4),H0_3(2,4),H0_3(3,4),scale*H0_3(1,1),scale*H0_3(2,1),scale*H0_3(3,1),'c')
quiver3(H0_3(1,4),H0_3(2,4),H0_3(3,4),scale*H0_3(1,2),scale*H0_3(2,2),scale*H0_3(3,2),'c')
quiver3(H0_3(1,4),H0_3(2,4),H0_3(3,4),scale*H0_3(1,3),scale*H0_3(2,3),scale*H0_3(3,3),'c')
%Plotting frame attached to the end-effector red
quiver3(P0(1),P0(2),P0(3),scale*H0_3(1,1),scale*H0_3(2,1),scale*H0_3(3,1),'r')
quiver3(P0(1),P0(2),P0(3),scale*H0_3(1,2),scale*H0_3(2,2),scale*H0_3(3,2),'r')
quiver3(P0(1),P0(2),P0(3),scale*H0_3(1,3),scale*H0_3(2,3),scale*H0_3(3,3),'r')
% Configure the axes
axis equal % Important! Scales all three axis the same so the robot isn't squished and stretched
grid on % Draw a light grid in the background of the figure axes
xlabel('X_0 (m)')
ylabel('Y_0 (m)')
zlabel('Z_0 (m)')
end

