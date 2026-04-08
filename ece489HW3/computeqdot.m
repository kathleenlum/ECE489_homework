function q_dot = computeqdot()
q = [pi/3; -pi/4; pi/2];
p0_dot = [0; 1; 1;];
j = computeJacobian(q, sym('L'));

q_dot = inv(j)*p0_dot;
code = latex(q_dot);
end