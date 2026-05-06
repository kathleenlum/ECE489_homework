function dJwall = fcn_dJwall(q, dq)
% return the time derivative of the Jacobian
p = get_params();
params = p.params;

dJ = fcn_dJ4(q, dq, params); % Time derivative of Jacobian
dJwall = -dJ(1, :); % time derivative of just the wall is the x row or the first row