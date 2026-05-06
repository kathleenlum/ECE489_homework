function F_ext = fcn_ExternalForce(t,X,p)

params = p.params;

q = X(1:3);
dq = X(4:6);

% External force at the end effector
F_ext = [0;0;0];

% end effector position and velocity
J  = fcn_J4(q, params);

curr_X = fcn_p4(q,params);
curr_Xdot = J*dq;

% wall characteristics
K_wall = 10000;   % Spring stiffness or squishiness (N/m)
B_wall = 500;     % Damping to avoid the bounce (Ns/m)

% if wall squished
if (curr_X(1) > 0.3)
    depth = curr_X(1) - 0.3; % amount the soft contact is squished by
    % spring damping force: F - kx - bv
    % points in the negative direction because it is repelling the end
    % effector not sucking it in
    Fx = -1*(K_wall * depth + B_wall * curr_Xdot(1)); % Apply spring-damping force
    if Fx > 0 % Don't want the wall to apply a pulling in force
       Fx = 0;
    end

    F_ext(1) = Fx
end

end