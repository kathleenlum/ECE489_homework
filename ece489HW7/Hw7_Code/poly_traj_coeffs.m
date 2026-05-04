function coeffs = poly_traj_coeffs(q_ds, ts)
t0 = ts(1);
t1 = ts(2);
t2 = ts(3);
t3 = ts(4);

% Initialize output: 14 coefficients for each of the 3 joints
coeffs = zeros(14, 3);

% =========================================================================
% Replace A and b with the expressions to represent the constraint
% equations for the coefficients for the concatenated polynomial
A = [% Position constraints (Rows 1-6)
     1 t0 t0^2  t0^3  t0^4  0  0    0      0      0  0    0      0       0 ;   % q1(t0)
     1 t1 t1^2  t1^3  t1^4  0  0    0      0      0  0    0      0       0 ;   % q1(t1)
     0  0  0     0     0    1 t1  t1^2   t1^3    0  0    0      0       0 ;   % q2(t1)
     0  0  0     0     0    1 t2  t2^2   t2^3    0  0    0      0       0 ;   % q2(t2)
     0  0  0     0     0    0  0    0      0      1 t2  t2^2   t2^3    t2^4;  % q3(t2)
     0  0  0     0     0    0  0    0      0      1 t3  t3^2   t3^3    t3^4;  % q3(t3)
     
     % Velocity constraints (Rows 7-10)
     0  1 2*t0  3*t0^2 4*t0^3 0  0    0      0      0  0    0      0       0 ;  % v1(t0)=0
     0  1 2*t1  3*t1^2 4*t1^3 0 -1 -2*t1  -3*t1^2   0  0    0      0       0 ;  % v1(t1)=v2(t1)
     0  0  0     0     0    0  1  2*t2   3*t2^2   0 -1 -2*t2  -3*t2^2 -4*t2^3; % v2(t2)=v3(t2)
     0  0  0     0     0    0  0    0      0      0  1  2*t3   3*t3^2  4*t3^3; % v3(t3)=0
     
     % Acceleration constraints (Rows 11-14)
     0  0  2    6*t0  12*t0^2 0  0    0      0      0  0    0      0       0 ;  % a1(t0)=0
     0  0  2    6*t1  12*t1^2 0  0   -2     -6*t1    0  0    0      0       0 ;  % a1(t1)=a2(t1)
     0  0  0     0     0    0  0    2      6*t2    0  0   -2     -6*t2  -12*t2^2;% a2(t2)=a3(t2)
     0  0  0     0     0    0  0    0      0      0  0    2      6*t3   12*t3^2];% a3(t3)=0
b = zeros(14,1);
% =========================================================================

for i = 1:3
    % the current joint
    qd1 = q_ds(i, 1);
    qd2 = q_ds(i, 2);
    qd3 = q_ds(i, 3);
    qd4 = q_ds(i, 4);
    
    % b vector -- based off of continuity and boundary constraints
    b = [qd1; qd2; qd2; qd3; qd3; qd4; 0; 0; 0; 0; 0; 0; 0; 0];
    
    % Solve: coeffs_vector = A \ b
    coeffs(:, i) = A \ b;
end
end
