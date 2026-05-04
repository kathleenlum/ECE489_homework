function [F_ext, p_ee] = fcn_ExternalForce(t,X,p)
%Function returns the external force and the point of contact pc

%Position of the end-effector
p_ee = fcn_p4(X(1:3,1),p.params); 

%External force applied to end-effector
F_ext = [0; 0; 0];
if (t > 2)
    F_ext = [0; 0; 0];
end

end