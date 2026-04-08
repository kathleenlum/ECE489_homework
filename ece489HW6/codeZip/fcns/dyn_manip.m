function dXdt = dyn_manip(t,X,p)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2025
%
% Calls functions to compute components of the equations of motion and then
% uses forward dynamics to solve for the time derivative of the robot state
% vector so it can be used in a simulation.

%Simulation_time = t %current simulation time

%Robot parameters
params = p.params;

%Current state
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector

%Calculating the matrices for the equation of motion:
De = fcn_De(q,params);      %Inertia metrix
Ce = fcn_Ce(q,dq,params);   %Coriolis matrix
Ge = fcn_Ge(q,params);      %Gravity vector
Be = fcn_Be(q,params);      %Torque selection (identity)

%Motor reflected inertia:
%{
J_RD = [1, 0, 0;
    0, 1, 0;
    0, 1, 1]; % Remotely driven linkage
De_m = J_RD.'*diag(p.N.^2 .* p.Ir)*J_RD;
%}
De_m = zeros(3,3); % Rotor inertia is ignored for this homework

%Joint friction model: Coulomb plus viscous
%Obs: The hyperbolic tangent function models the friction profile
Coulomb = p.friction(1:3,1);
Viscous = p.friction(4:6,1);
tau_friction = - diag(Coulomb)*tanh(20*dq) - diag(Viscous)*dq;

%Controller function that computes the joint torques to be applied
p_estimated = get_params_estimated();
tau = fcn_controller(t,X,p_estimated);

%Solving for the joint accelerations
ddq = (De + De_m) \ (Be*tau + tau_friction - Ce*dq - Ge); 

dXdt = [dq; ddq];
