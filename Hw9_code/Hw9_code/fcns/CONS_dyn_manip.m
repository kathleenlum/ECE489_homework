function [dXdt, F_c] = CONS_dyn_manip(t,X,p)
% Authors: Yanran Ding and Joao Ramos
% Edited by Justin Yim 2023
%Simulation_time = t %current simulation time

%Robot parameters
params = p.params;

%Actuators reflected inertia matrix:
N = 25;
Ir = 1.4e-5;
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir];];

%Current state
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector
%Calculating the matrices for the equation of motion:
De = fcn_De(q,params)+M_DH; %Inertia matrix
Ce = fcn_Ce(q,dq,params);   %Coriolis matrix
Ge = fcn_Ge(q,params);      %Gravity vector
Je = fcn_J4(q,params);      %End-effector Jacobian
dJe = fcn_dJ4(q,dq,params); %End-effector Jacobian time derivative
%Joint friction model: Coulomb plus viscous
%Obs: The hyperbolic tangent function models the friction profile
tau_Coulomb = p.friction(1:3,1);
b_viscous = p.friction(4:6,1);
friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;

%External force applied by human
F_ext = fcn_ExternalForce(t,X,p);

%Controller function that computes the joint torques to be applied
tau = fcn_controller(t,X,p,1);

%Solve for constrained dynamics:
ddq = [0; 0; 0]; % CHANGE THIS
Jwall = fcn_Jwall(q);
dJwall = fcn_dJwall(q,dq);

% torque with everything but the wall
u = tau + friction + Je'*F_ext - Ce*dq - Ge;

F_c = -1*(Jwall*(De \ u) + dJwall*dq)/(Jwall*(De\Jwall.'));         % CHANGE THIS

if(F_c < 0)
    F_c = 0;
end

ddq = De \ (u + Jwall.'*F_c);
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
dXdt = [dq; ddq;];
