function dXdt = dyn_manip(t,X,p)
%Simulation_time = t %current simulation time

%Robot parameters
params = p.params;

%Actuators reflected inertia matrix:
N = 25;
Ir = 1.4e-5;
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir];];

%--------------------------------------------------------------------------
%MANIPULATOR 1-------------------------------------------------------------
%Leader robot:
%Current state
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector
%Calculating the matrices for the equation of motion:
De = fcn_De(q,params);      %Inertia matrix due to links
Ce = fcn_Ce(q,dq,params);  %Coriolis matrix
Ge = fcn_Ge(q,params);      %Gravity vector
Be = fcn_Be(q,params);      %Torque selection (identity)
Jc = fcn_J4(q,params);      %End-effector Jacobian
%Joint friction model: Coulomb plus viscous
%Obs: The hyperbolic tangent function models the friction profile
tau_Coulomb = p.friction(1:3,1);
b_viscous = p.friction(4:6,1);
friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;

%External force
F_ext = fcn_ExternalForce(t,X,p);

%Controller function that computes the joint torques to be applied
p_estimated = get_params_estimated();
tau = fcn_controller(t,X,p_estimated);

%Solving for the joint accelerations
ddq = (De + M_DH) \ (Be*tau + friction + Jc'*F_ext - Ce*dq - Ge);

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Return acceleration of both manipulators
dXdt = [dq; ddq];
