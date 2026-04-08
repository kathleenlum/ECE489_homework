function Jout = computeJacobian(angles, links)
syms L
syms t1 t2 t3
t = [t1; t2; t3];
theta1 = t(1);
theta2 = t(2);
theta3 = t(3);

p0 = [L*cos(theta1)*(cos(theta2+theta3)+ cos(theta2));
      L*sin(theta1)*(cos(theta2+theta3)+cos(theta2));
      L*(1-sin(theta2)-sin(theta2+theta3));];

J = jacobian(p0, t);
J = simplify(J);

Jout = subs(subs(J, t, angles), L, links);
code = latex(Jout);

deter = latex(simplify(det(J)));
disp(deter);

end