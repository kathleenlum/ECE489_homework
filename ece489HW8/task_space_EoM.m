function [M_T, C_T, G_T] = task_space_EoM(t,X,p_hat)
% Author: Justin Yim, 2023
%
% INPUT:
% t: time in seconds
% X: robot state [theta_1; theta_2; theta_3; theta_dot_1; theta_dot_2;
% theta_dot 3]
% p_hat: estimated robot parameters
%
% OUTPUT
% M_T: task-space inertia matrix
% C_T: task-space Coriolis matrix
% G_T: task-space Gravity terms

q = X(1:3);
dq = X(4:6);

Jc = fcn_J4(q,p_hat.params);      %End-effector Jacobian
De_hat = fcn_De(q,p_hat.params);        %Joint-space mass matrix
Ce_hat = fcn_Ce(q,dq,p_hat.params);     %Joint-space Coriolis matrix
Ge_hat = fcn_Ge(q,p_hat.params);        %Joint-space Gravity terms
Jc_dot = fcn_dJ4(q,dq,p_hat.params);    %End-effector Jacobian derivative

J_inv = pinv(Jc);
%task-space matrices derived 
M_T = J_inv.' * De_hat * J_inv;
C_T = J_inv.' * (Ce_hat - De_hat * J_inv * Jc_dot) * J_inv;
G_T = J_inv.' * Ge_hat;
