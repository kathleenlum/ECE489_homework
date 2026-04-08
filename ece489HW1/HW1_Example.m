% HW1_Example.m
% Written by Joao Ramos
% Modified by Justin Yim 2025

% Note that this example does NOT follow the Denavit-Hartenberg convention
% that we will use in later assignments.

close all; % close all figure windows (close all plotted output)

%% Geometry and Transformations

%Joint angles
theta1 = pi/4;%-pi/4;
theta2 = 7*pi/4;%-3*pi/6;

%System Parameters
L1 = 0.5;
L2 = 0.6;

%First joint kinematics
R0_1 = [[cos(theta1) -sin(theta1) 0]; % 1st joint rotation matrix: rotation of frame 1 with respect to frame 0
       [sin(theta1) cos(theta1) 0];
       [0 0 1];]; 
d0_1 = [0.3; 0.4; 0]; %[0; 0; 0]; % displacement of frame 1 w.r.t. frame 0
H0_1 = [[R0_1 d0_1]; % homogeneous transformation matrix of frame 1 wr.t. frame 0
       [0 0 0 1];];
   
%Second joint kinematics
R1_2 = [[cos(theta2) -sin(theta2) 0];
        [sin(theta2) cos(theta2) 0];
        [0 0 1];];
        %[[1 0 0]; % 2nd joint rotation matrix: rotation of frame 2 w.r.t. frame 1
       %[0  cos(theta2) sin(theta2)];
       %[0 -sin(theta2) cos(theta2)];];
d1_2 = [0; 0; L1;]; % displacement of frame 2 w.r.t. frame 1
H1_2 = [[R1_2 d1_2]; % homogeneous transformation matrix of frame 2 w.r.t. frame 1
       [0 0 0 1];]; 

% HTM to world frame (frame 0)
H0_2 = H0_1*H1_2; % homogeneous transformation matrix of frame 2 w.t.t frame 0

% Points in the world frame (frame 0)
O0_1 = H0_1*[0; 0; 0; 1]; % frame 1 origin w.r.t. world frame
O0_2 = H0_2*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame
P0 = H0_2*[0; L2; 0; 1]; % End-effector position w.r.t world frame

% Points to draw (frame 1 origin, frame 2 origin, end-effector)
pts = [O0_1, O0_2];%, P0];

%% Plotting

figure(1) % Open figure window 1
clf % Clear figure window 1 (erase everything in it)

% Draw the robot stick figure
plot3(pts(1,:),pts(2,:),pts(3,:),'.-k','LineWidth',3,'MarkerSize',20)
% plot3 draws a sequence of line segments connecting points in 3D space;
% the .-k option says to use a dot at each point (.), draw a solid line
% between subsequent points (-), and use black for the color (k). See the
% online MATLAB documentation for more details (it's very convenient).

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

%Plotting frame attached to the end-effector red
%quiver3(P0(1),P0(2),P0(3),scale*H0_2(1,1),scale*H0_2(2,1),scale*H0_2(3,1),'r')
%quiver3(P0(1),P0(2),P0(3),scale*H0_2(1,2),scale*H0_2(2,2),scale*H0_2(3,2),'r')
%quiver3(P0(1),P0(2),P0(3),scale*H0_2(1,3),scale*H0_2(2,3),scale*H0_2(3,3),'r')

% Configure the axes
axis equal % Important! Scales all three axis the same so the robot isn't squished and stretched
grid on % Draw a light grid in the background of the figure axes
set(gca,'xlim',[-1.3,1.3],'ylim',[-1.3,1.3],'zlim',[0,2]) % adjust the plot space bounds
xlabel('X_0 (m)')
ylabel('Y_0 (m)')
zlabel('Z_0 (m)')
