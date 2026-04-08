function Animate_Planar_Manipulator(time, X, record)
% Written by Justin Yim 2023
% Based on code by Joao Ramos

% Parameters
global g L1 L2 m1 m2

dtA = 0.02;             %Time step of animation
timeA = 0:dtA:time(end);   %Time vector for animation
Xe = [];

if (record)
    v = VideoWriter('Planar_Manip.avi');
    open(v)
end

f = figure(10);
clf
set(f, 'doublebuffer', 'on');
h_arm = plot([0 0 L1],[0 0 L2],'k','LineWidth',4);
hold on
h_trace = plot(0, 0, 'm', 'LineWidth', 2);
plot([-0.5 0.5],[0 0],'k','LineWidth',1)        %Draw floor

h_FE = quiver(0,0,0,0,'r','LineWidth',2);

axis equal
axis((L1+L2)*[-1 1 -1 1])
h_title = title('Time = 0');
xlabel('X [m]')
ylabel('Y [m]')
grid on
hold off

tic
for k = 1:length(timeA)     
    %Robot State
    q1 = interp1(time,X(:,1),timeA(k));
    q2 = interp1(time,X(:,2),timeA(k));
    
    X1 = L1*cos(q1);
    Y1 = L1*sin(q1);    
    X2 = L1*cos(q1) + L2*cos(q1 + q2);
    Y2 = L1*sin(q1) + L2*sin(q1 + q2);
    Xe = [Xe [X2; Y2]];
    
    FE = ExternalForce(timeA(k));
    
    %Drawing Manipulator
    set(h_arm, 'XData', [0, X1 X2], 'YData', [0, Y1, Y2]);
    set(h_trace, 'XData', Xe(1,:), 'YData', Xe(2,:));
    set(h_FE, 'XData', X2, 'YData', Y2, 'UData', FE(1)/20, 'VData', FE(2)/20)
    set(h_title, 'String', ['Time =' num2str(timeA(k),'%6.2f') 's'])
    drawnow
    
    % Timing 
    if ~record
        pause(dtA-toc)
        tic
    else
        F = getframe(f);
        writeVideo(v,F)
    end
end

%Closing file
if(record)
    close(v)
end