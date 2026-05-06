function Jwall = fcn_Jwall(q)

p = get_params();
params = p.params;

J = fcn_J4(q,params);
Jwall = -J(1, :); % we want to take the first row of the jacobian
% the jacobian of the wall is only respective to the x distance
% negate the jacobian to get it w/r/t joint coordinates