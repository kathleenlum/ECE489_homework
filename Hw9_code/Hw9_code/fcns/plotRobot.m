function h = plotRobot(X,p,RobotColor,varargin)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Plot the CRS robot arm in the configuration given by joint vector X
%
% INPUT
% X: vector of joint angles in radians (length 3)
% p: robot parameter struct
% RobotColor: 1x3 RGB color vector (all values between 0 and 1 inclusive)
% h (optional input): cell array of graphics object handles if this
%   has already been drawn once before.  h is also the function output.

if nargin >= 4
    h = varargin{1}; % Plot handles
else
    h = {};
end

params = p.params;

% Kinematics --------------------------------------------------------------
ang = 0:0.1:2*pi;
q = X(1:3);
p1 = [0 0 0]';
p2 = fcn_p2(q,params);
p3 = fcn_p3(q,params);
%p4 = fcn_p4(q,params);

T01 = fcn_T01(q,params); % Robot link 1
T02 = fcn_T02(q,params); % Robot link 2
T03 = fcn_T03(q,params); % Robot link 3
T04 = fcn_T04(q,params); % Gripper

%Vetical rod at end-effector
d = 2.65*0.0254;
height = 6*0.0254;
dia = 0.75*0.0254;
p4 = fcn_p4(q,params);
p_aux = [p4; 0] + T01*[d; 0; 0; 1];
Tee = [T01(1:4,1:3) p_aux];

chain = [p1 p2 p3 p4];% p_aux(1:3)];

% Plotting ----------------------------------------------------------------
if isempty(h)
    % Draw a new robot object
    hold on
    h{1} = plot3(chain(1,:),chain(2,:),chain(3,:),...
        '-k','LineWidth',4); % Robot stick figure
    h{2} = fill3(0.5*cos(ang), 0.5*sin(ang), 0*cos(ang),...
        [0.5 0.5 0.5],'facealpha',0.9); %Tabletop
    h{3} = DrawCube([5; 5; 3.32]*0.0254,[0; 0; 1.66]*0.0254,...
        RobotColor,eye(4)); %Robot base
    h{4} = DrawCube([4; 4; 6.68]*0.0254,[0; 0; 6.66]*0.0254,...
        RobotColor,T01); %Robot link 1
    h{5} = DrawCube([10; 2.5; 2.5]*0.0254,[5; 0; 0]*0.0254,...
        RobotColor,T02); %Robot link 2
    h{6} = DrawCube([10; 1.5; 0.5]*0.0254,[5; 0; 0]*0.0254,...
        RobotColor,T03); %Robot link 3
    %h{7} = DrawCube([dia; dia; height],[0; 0; 0],...
    %    RobotColor,Tee); % Vertical rod at end effector
    h{8} = DrawFrame(eye(4), 0.3); % Origin frame, frame 0
    h{9} = DrawFrame(T04, 0.1); % End effector frame, frame 4
    hold off
else
    % Update a previously drawn robot object
    set(h{1},'XData',chain(1,:),'YData',chain(2,:),'Zdata',chain(3,:));
    h{3} = DrawCube([5; 5; 3.32]*0.0254,[0; 0; 1.66]*0.0254,...
        RobotColor,eye(4), h{3});
    h{4} = DrawCube([4; 4; 6.68]*0.0254,[0; 0; 6.66]*0.0254,...
        RobotColor,T01, h{4});
    h{5} = DrawCube([10; 2.5; 2.5]*0.0254,[5; 0; 0]*0.0254,...
        RobotColor,T02, h{5});
    h{6} = DrawCube([10; 1.5; 0.5]*0.0254,[5; 0; 0]*0.0254,...
        RobotColor,T03, h{6});
    %h{7} = DrawCube([dia; dia; height],[0; 0; 0],...
    %    RobotColor,Tee,h{7}); 
    h{8} = DrawFrame(eye(4), 0.3, h{8});
    h{9} = DrawFrame(T04, 0.1, h{9});
end

end
