function animateRobot(tin,Xin,p,record)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Animate the CRS robot arm along a trajectory from, for example, a
% simulation.
%
% INPUT
% tin: nx1 vector of times in seconds
% Xin: nx3 vector of joint angles (in radians) at times in tin
% p: robot parameter struct
% record: boolean to record a video (1) or not (0)

dtA = 0.05;             %Time step of animation
timeA = 0:dtA:tin(end); %Time vector for animation with constant stepping

% record = 0;           %Set to 1 to record video and 0 to not
if (record)
    v = VideoWriter('CRS_Robot.avi');
    open(v)
end

params = p.params;

% Figure setup ------------------------------------------------------------
f = figure(10);
clf

%External force
Fsc = 0.03; % Force plotting scale
h_force = quiver3(0,0,0, 0,0,0, 'm','LineWidth',2);

%Obstacle 1:
hold on
DrawCube([0.1; 0.1; 0.1],[0; 0.4; 0.05],[0.5 0.5 0.5],eye(4)); 
ang = 0:0.1:2*pi;
hold on
fill3(0.025*cos(ang),0.4+0.025*sin(ang),0.1+0*sin(ang),1*[1 1 1])

%Obstacle 2:
w = 1.5*0.0254; %gap width
T = [[rz(pi/2) [0.35; 0; 0.1]];
     [0 0 0 1];];
hold on
DrawCube([0.2; 0.1; 0.2],[0; 0.05+w/2; 0.0],[0.5 0.5 0.5],T);
hold on
DrawCube([0.2; 0.1; 0.2],[0; -(0.05+w/2); 0.0],[0.5 0.5 0.5],T);

%Obstacle 3:
hold on
DrawCube([0.1; 0.1; 0.3],[0; -0.4; 0.15],[0.5 0.5 0.5],eye(4));
hold on
fill3(0.025*cos(ang),-0.4+0.025*sin(ang),0.3+0*sin(ang),[1 0 0])

% Desired points
%hold on
%plot3([0.4,0.2,0.2,0.4],[0,0,0,0],[0.2,0.2,0.5,0.5],'or');

% End effector trace
hold on
x_trace = NaN(3,length(timeA));
h_trace = plot3(0,0,0,'r');

% Side view for this homework
%view(0,0)

set(f, 'doublebuffer', 'on');
grid on
axis equal
axis([-0.6 0.6 -0.6 0.6 -0.3 0.9])
xlabel('x [m]','fontsize',12)
ylabel('y [m]','fontsize',12)
zlabel('z [m]','fontsize',12)

% Enlarge figure
%set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.05, 0.1, 0.4, 0.7]);

h_title = title('0 s');
h = {}; % Graphics handles

% Animating ---------------------------------------------------------------
tic
for k = 1:length(timeA) 
    
    %Get the joint angles at the current time
    theta1 = interp1(tin,Xin(1,:),timeA(k));
    theta2 = interp1(tin,Xin(2,:),timeA(k));
    theta3 = interp1(tin,Xin(3,:),timeA(k));
    dtheta1 = interp1(tin,Xin(4,:),timeA(k));
    dtheta2 = interp1(tin,Xin(5,:),timeA(k));
    dtheta3 = interp1(tin,Xin(6,:),timeA(k));
    
    [F_ext, pc] = fcn_ExternalForce(timeA(k),...
        [theta1;theta2;theta3;dtheta1;dtheta2;dtheta3],p); %External force
    % and end-effector position
    
    %Plot the robot
    h = plotRobot([theta1; theta2; theta3],p,[0 0.5 1],h);
    
    % Plot the external force
    set(h_force,'Xdata',pc(1),'Ydata',pc(2),'Zdata',pc(3),...
        'Udata',F_ext(1)*Fsc,'Vdata',F_ext(2)*Fsc,'Wdata',F_ext(3)*Fsc)
    
    % Plot the trace
    x_trace(:,k) = pc;
    set(h_trace, 'Xdata', x_trace(1,:), 'YData', x_trace(2,:), ...
        'Zdata', x_trace(3,:))
    
    % Update the title
    set(h_title,'string',['t = ',num2str(timeA(k),'%6.2f'),'s'])
    
    if(record)
        %Save a video frame
        drawnow
        F = getframe(f);
        writeVideo(v,F)
    else
        %Handle animation timing
        drawnow
        pause(dtA-toc)
        tic
    end
end
%Closing file
if(record)
    close(v)
end

end







