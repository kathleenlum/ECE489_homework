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
angles = fsolve(@(q) CRS_FK(q) - p_des, q_init);
q = angles; % Assign the computed joint angles to the output variable
%q = [0;0;0]; % replace this

end