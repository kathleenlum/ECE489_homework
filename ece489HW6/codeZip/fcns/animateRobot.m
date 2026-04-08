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

% Figure setup ------------------------------------------------------------
f = figure(10);
clf
set(f, 'doublebuffer', 'on');
plot3(0,0,0,'.k')
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
    
    %Plot the robot
    h = plotRobot([theta1; theta2; theta3],p,[0 0.5 1],h);
    
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







