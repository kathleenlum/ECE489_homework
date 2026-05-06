function [value, isterminal, direction] = wall_contact(t,X)
%{
 return the distance from the wall to the end effector in units of meters. 
This value should be positive when the end effector is not in contact with the wall
and be negative when the end effector is through the wall. 
When this value becomes zero, the robot transitions from free to contact mode. 
%}
p = get_params();
params = p.params;

q = X(1:3);            %Joint angles vector
dq = X(4:6);           %Joint velocities vector
p = fcn_p4(q,params);  %End-effector spatial position

value      = 0.35-p(1);%1; %Guard function, contact occurs when: value = 0. CHANGE THIS
isterminal = 1; %Stop the ode45 integration
direction  = -1;
end