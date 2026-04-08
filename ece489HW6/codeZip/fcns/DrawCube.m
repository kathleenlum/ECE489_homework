function h = DrawCube(size,offset,color,HTM,varargin)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%
% Draw a rectangular prism (not necessarily a cube)
%
% INPUT
% size: vector of box 
% p: robot parameter struct
% RobotColor: 1x3 RGB color vector (all values between 0 and 1 inclusive)
% h (optional input): cell array of graphics object handles if this
%   has already been drawn once before.  h is also the function output.

if nargin >= 5
    h = varargin{1};
else
    h = {};
end

W = size(1);
H = size(2);
L = size(3);

P1 = [offset; 0] + [W/2 H/2 L/2 1]';
P2 = [offset; 0] + [W/2 -H/2 L/2 1]';
P3 = [offset; 0] + [-W/2 -H/2 L/2 1]';
P4 = [offset; 0] + [-W/2 H/2 L/2 1]';
P5 = [offset; 0] + [W/2 H/2 -L/2 1]';
P6 = [offset; 0] + [W/2 -H/2 -L/2 1]';
P7 = [offset; 0] + [-W/2 -H/2 -L/2 1]';
P8 = [offset; 0] + [-W/2 H/2 -L/2 1]';

Pts = HTM*[P1 P2 P3 P4];

% Plotting ----------------------------------------------------------------
if isempty(h)
    % Draw a new rectangular prism
    hold on
    h{1} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    Pts = HTM*[P3 P2 P6 P7];
    h{2} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    Pts = HTM*[P4 P1 P5 P8];
    h{3} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    Pts = HTM*[P4 P3 P7 P8];
    h{4} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    Pts = HTM*[P2 P1 P5 P6];
    h{5} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    Pts = HTM*[P5 P6 P7 P8];
    h{6} = fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
    hold off
else
    % Update a previously drawn rectangular prism
    set(h{1},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
    Pts = HTM*[P3 P2 P6 P7];
    set(h{2},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
    Pts = HTM*[P4 P1 P5 P8];
    set(h{3},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
    Pts = HTM*[P4 P3 P7 P8];
    set(h{4},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
    Pts = HTM*[P2 P1 P5 P6];
    set(h{5},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
    Pts = HTM*[P5 P6 P7 P8];
    set(h{6},'XData',Pts(1,:),'YData',Pts(2,:),'ZData',Pts(3,:));
end


end