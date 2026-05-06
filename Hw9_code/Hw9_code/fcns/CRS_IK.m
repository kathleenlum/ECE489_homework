function q = CRS_IK(p_des, q_init)
% Solve the inverse kinematics problem for the CRS robot arm
% OUTPUT:
% q: 3x1 column vector of joint angles in radians
% INPUTS:
% p_des: 3x1 column vector desired end effector coordinates in frame 0
% (all units of meters)
% q_init: 3x1 column vector of initial joint-space guess (all units of
% radians)
% Your code goes here
function F = solveforpoint(thetas)
F = CRS_FK(thetas) - p_des
end
q = fsolve(@solveforpoint, q_init)
end