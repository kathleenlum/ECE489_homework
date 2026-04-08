function p = get_params()

% parameters for matrix calculations

% g Gravity constant
% l1,l2,l3 Length of link 1-3
% M1,M2,M3 Mass of link 1-3
% J1,J2,J3 Moment of Inertia of Link 1-3

g = 9.81; %m/s^2

l1 = 10*2.54/100; % unit: meter
l2 = l1;
l3 = l2;

M1 = 3.12; % unit: Kilogram
M2 = 1.30;
M3 = 1.14;

%Links inertia tensors (Note: J_ij = J_ji)
%Link 1
%I1 = [[J1_xx J1_xy J1_xz];
%      [J1_xy J1_yy J1_yz];
%      [J1_xz J1_yz J1_zz];]; kg*m^2
       
J1_xx = 0.0155; 
J1_yy = 0.0096; 
J1_zz = 0.0150; 
J1_xy = 3.4e-6; 
J1_xz = 1.3e-4; 
J1_yz = 1.5e-5;

%Link 2
J2_xx = 0.0021; 
J2_yy = 0.0114; 
J2_zz = 0.0104; 
J2_xy = -8e-7; 
J2_xz = 5.5e-5; 
J2_yz = 0;

%Link 3
J3_xx = 0.0010; 
J3_yy = 0.0111; 
J3_zz = 0.0106; 
J3_xy = 0; 
J3_xz = 0; 
J3_yz = 0;

%Friction model Coulomb + viscous
tau1_Coulomb = 0.1;
tau2_Coulomb = 0.1;
tau3_Coulomb = 0.1;
b1_viscous = 0.01;
b2_viscous = 0.01;
b3_viscous = 0.01;

params = [g, l1, l2, l3, M1, M2, M3, J1_xx, J1_yy, J1_zz, J1_xy, J1_xz,...
    J1_yz, J2_xx, J2_yy, J2_zz, J2_xy, J2_xz, J2_yz, J3_xx, J3_yy, J3_zz,...
    J3_xy, J3_xz, J3_yz];

% Rotor inertia
Ir1 = 2E-5;
Ir2 = 2E-5;
Ir3 = 2E-5;

% Gear ratios
N1 = 25;
N2 = 25;
N3 = 25;

p.friction = [tau1_Coulomb; tau2_Coulomb; tau3_Coulomb; b1_viscous; b2_viscous; b3_viscous];
p.params = params;
p.Ir = [Ir1, Ir2, Ir3];
p.N = [N1, N2, N3];
p.N_animate = 30;   % for animation time smoothness
















