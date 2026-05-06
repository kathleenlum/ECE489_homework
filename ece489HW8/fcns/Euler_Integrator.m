function [time, X] = Euler_Integrator(f, tspan, X0, DeltaT)
% Starter code by Justin Yim 2023
%
% INPUTS:
% f: a Matlab function handle that accepts the inputs t (scalar) and X (an
%   n x 1 vector) and computes X_dot = f(t, X) for a dynamical system
%   (see Planar_Manipulator.m).
% tspan: an array of length 2 with the start time and end time for the
%   simulation in second.
% x0: vector of initial conditions
% DeltaT: time step size in seconds
% 
% OUTPUTS:
% time: T_steps by 1 array of simulation time samples in seconds
% X: T_steps by n array of simulation states at the time samples

time = (tspan(1):DeltaT:tspan(end)).'; % time sample points
T_steps = numel(time);      % how many steps

X_dot0 = f(tspan(1), X0);   % Call the dynamics function once
n = length(X_dot0);         % number of states (2x coordinates) in system
X = zeros(T_steps, n);      % array of states to return
X(1,:) = X0;                % The first point in X is the initial condition

for ii = 2:T_steps
    X(ii,:) = X(ii-1,:).' + f(time(ii-1), X(ii-1,:).')*DeltaT;
end

end