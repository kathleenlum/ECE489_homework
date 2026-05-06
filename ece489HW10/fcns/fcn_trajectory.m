function [qd, qd_dot, force] = fcn_trajectory(t,p)
% qd = xd: 3x1 desired Cartesian position [x; y; z] in meters
% qd_dot = xd_dot: 3x1 desired Cartesian velocity [vx; vy; vz] in m/s


%Desired trajectory
qd = [0; 0; 0]; %Desired joint angles at time t
qd_dot = [0; 0; 0]; %Desired joint velocities at time t
if t < 1.0
    %First target point
    qd = [0.4; 0; 0.15];
else
    %Second target point (Step jump occurs at t = 1s)
    qd = [0.2; 0.2; 0.35];
end


end