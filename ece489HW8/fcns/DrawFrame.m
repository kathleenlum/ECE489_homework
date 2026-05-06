function h = DrawFrame(HTM, scale, varargin)
% Written by Justin Yim 2023, based on code by Yanran Ding and Joao Ramos
%
% Draw a 3D coordinate frame using red for x, green for y, and blue for z
%
% INPUT
% HTM: a spatial (not planar) homogeneous transform matrix between the
%   Matlab axes origin frame and the frame to be drawn.
% scale: length at which to draw each of the axes (meters by default)
% h (optional input): cell array of graphics object handles if this
%   has already been drawn once before.  h is also the function output.

if nargin >= 3
    h = varargin{1};
else
    h = {};
end

if isempty(h)
    hold on
    h{1} = quiver3(HTM(1,4),HTM(2,4),HTM(3,4),...
        scale*HTM(1,1),scale*HTM(2,1),scale*HTM(3,1),'r');
    h{2} = quiver3(HTM(1,4),HTM(2,4),HTM(3,4),...
        scale*HTM(1,2),scale*HTM(2,2),scale*HTM(3,2),'g');
    h{3} = quiver3(HTM(1,4),HTM(2,4),HTM(3,4),...
        scale*HTM(1,3),scale*HTM(2,3),scale*HTM(3,3),'b');
    hold off
else
    set(h{1},'XData',HTM(1,4),'YData',HTM(2,4),'ZData',HTM(3,4),...
        'UData',scale*HTM(1,1),'VData',scale*HTM(2,1),'WData',scale*HTM(3,1));
    set(h{2},'XData',HTM(1,4),'YData',HTM(2,4),'ZData',HTM(3,4),...
        'UData',scale*HTM(1,2),'VData',scale*HTM(2,2),'WData',scale*HTM(3,2));
    set(h{3},'XData',HTM(1,4),'YData',HTM(2,4),'ZData',HTM(3,4),...
        'UData',scale*HTM(1,3),'VData',scale*HTM(2,3),'WData',scale*HTM(3,3));
end