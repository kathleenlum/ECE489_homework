%symbolic trajectory matrix calculation script
% written by Kathleen Lum (kl57)

syms t0 t1 t2 t3
syms qd1 qd2 qd3 qd4
syms a10 a11 a12 a13 a14 a20 a21 a22 a23 a30 a31 a32 a33 a34


A = [%position
     1 t0 t0^2  t0^3  t0^4  0  0    0      0      0  0    0      0       0 ;       % q1(t0)
     1 t1 t1^2  t1^3  t1^4  0  0    0      0      0  0    0      0       0 ;       % q1(t1)
     0  0  0     0     0    1 t1  t1^2   t1^3    0  0    0      0       0 ;       % q2(t1)
     0  0  0     0     0    1 t2  t2^2   t2^3    0  0    0      0       0 ;       % q2(t2)
     0  0  0     0     0    0  0    0      0      1 t2  t2^2   t2^3    t2^4;      % q3(t2)
     0  0  0     0     0    0  0    0      0      1 t3  t3^2   t3^3    t3^4;      % q3(t3)
     % velocity
     0  1 2*t0  3*t0^2 4*t0^3 0  0    0      0      0  0    0      0       0 ;      % v1(t0)=0
     0  1 2*t1  3*t1^2 4*t1^3 0 -1 -2*t1  -3*t1^2   0  0    0      0       0 ;      % v1(t1)-v2(t1)=0
     0  0  0     0     0    0  1  2*t2   3*t2^2   0 -1 -2*t2  -3*t2^2 -4*t2^3;    % v2(t2)-v3(t2)=0 
     0  0  0     0     0    0  0    0      0      0  1  2*t3   3*t3^2  4*t3^3;    % v3(t3)=0
     % acceleration
     0  0  2    6*t0  12*t0^2 0  0    0      0      0  0    0      0       0 ;      % a1(t0)=0
     0  0  2    6*t1  12*t1^2 0  0   -2     -6*t1    0  0    0      0       0 ;      % a1(t1)-a2(t1)=0
     0  0  0     0     0    0  0    2      6*t2    0  0   -2     -6*t2  -12*t2^2;   % a2(t2)-a3(t2)=0 
     0  0  0     0     0    0  0    0      0      0  0    2      6*t3   12*t3^2];   % a3(t3)=0

a = [a10; a11; a12; a13; a14; a20; a21; a22; a23; a30; a31; a32; a33; a34];
b = [qd1; qd2; qd2; qd3; qd3; qd4; 0; 0; 0; 0; 0; 0; 0; 0]; % Define the boundary conditions

disp(latex(A))
disp(latex(a))
disp('=')
disp(latex(b))
%coefficients = A \ b; % Solve for the coefficients