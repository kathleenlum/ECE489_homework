function p = CRS_FK(q)
% Solve the forwards kinematics problem for the CRS robot arm.
% OUTPUT:
% p: 3x1 column vector: end effector coordinates in frame 0 (all units of
% meters)
% INPUT:

% q: 3x1 column vector: joint coordinates (all units of radians)

% Your code goes here
a = [0 10*0.0254 10*0.0254];
al = [-sym(pi)/2 0 0];
d = [10*0.0254 0 0];

A1 = DH_HTM(a, al, d, q, 1); % H0_1
A2 = DH_HTM(a, al, d, q, 2); % H1_2
A3 = DH_HTM(a, al, d, q, 3); % H2_3

H0_1 = simplify(A1);
H0_2 = simplify(A1*A2);
H0_3 = simplify(A1*A2*A3);

% Points in the world frame (frame 0)
O0 = [0; 0; 0; 1];
O0_1 = H0_1*[0; 0; 0; 1]; % frame 1 origin w.r.t. world frame
O0_2 = H0_2*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame
O0_3 = H0_3*[0; 0; 0; 1]; % frame 2 origin w.r.t. world frame

P = H0_3*[0; 0; 0; 1]; % End-effector position w.r.t world frame
% displacement in y dir (ref figure) P0 = H0_3*P3

p = double(P(1:3));

end