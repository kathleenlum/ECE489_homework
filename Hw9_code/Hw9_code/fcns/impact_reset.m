function [X_plus, Fwall_imp] = impact_reset(X_minus)

p = get_params();
params = p.params;

q = X_minus(1:3); %Joint angles vector
dq_minus = X_minus(4:6); %Joint velocities vector

%Actuators reflected inertia matrix:
N = 25;
Ir = 1.4e-5;
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir];];
De = fcn_De(q,params)+M_DH; %Inertia matrix

%{
Jw = fcn_Jwall(q);

Fwall_imp = (-Jw*dq_minus) / (Jw * (De\Jw')); % Calculate the forces based on the wall contact

% 3. Calculate the new joint velocity (dq_plus)
dq_plus = dq_minus + De \ (Jw' * Fwall_imp);

X_plus = [q; dq_plus];%X_minus; % CHANGE THIS
%Fwall_imp = 0; % CHANGE THIS

%}
% --- Augmented Matrix Logic ---
% We solve for the post-impact velocity (dq_plus) and the impulse (L)
% satisfying: M*dq_plus - Jc'*L = M*dq_minus  (Momentum balance)
%             Jc*dq_plus = 0                 (Kinematic constraint)
Jc = fcn_Jwall(q);

A = [De, -Jc.'; 
     Jc,  0];

b = [De * dq_minus; 0];

% Solve the linear system
% Adding a tiny epsilon (1e-12) is safer than 1e-8 for precision
sol = A \ b;

% Extract the results
dq_plus = sol(1:3);      % New joint velocities
Fwall_imp = sol(4);      % The scalar impact impulse

% Update the state vector
X_plus = [q; dq_plus];
%{
disp('dq_plus:');
disp(dq_plus);
disp('Fwall_imp');
disp(Fwall_imp);
%}