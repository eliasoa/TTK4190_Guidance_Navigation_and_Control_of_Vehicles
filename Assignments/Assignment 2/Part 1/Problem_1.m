clear all
%% Real Ship
L = 161;                % length (m)
B = 21.8;               % beam (m)
H = 15.8;               % heighy (m)
m = 17.0677e6;          % mass (kg)
rho_m = m/(L*B*H);      % density of ship (kg/m^3)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
%% Problem 1a

xmin = -L/2;
xmax = L/2;
ymin = -B/2;
ymax = B/2;
zmin = -H/2;
zmax = H/2;
fun = @(x,y,z) y.^2 + z.^2 ;
I_CG_X = rho_m * integral3(fun,xmin,xmax,ymin,ymax,zmin,zmax);
fun = @(x,y,z) x.^2 + z.^2;
I_CG_Y = rho_m * integral3(fun,xmin,xmax,ymin,ymax,zmin,zmax);
fun = @(x,y,z) x.^2 + y.^2;
I_CG_Z = rho_m * integral3(fun,xmin,xmax,ymin,ymax,zmin,zmax);
%% Problem 1b
r_b_bg = [-3.7 0 H/2]';

% Paralell axis theorem 
I_CO_Z_PAT = [I_CG_X 0 0; 0 I_CG_Y 0; 0 0 I_CG_Z] + m*(r_b_bg.'*r_b_bg*eye(3) - r_b_bg*r_b_bg.');

% Using eq. 3.37 in Fossen only the element I_z^CG
I_CO_Z = I_CG_Z + m*(r_b_bg(1)^2 + r_b_bg(2)^2)

ratio_box_real = I_CO_Z/Iz

%% Problem 1c
syms m p q r x y z Ix Iy Iz Ixy Ixz Iyz

r_bg = [x; y; z];

nu2 = [p; q; r];

I_CG = [Ix -Ixy -Ixz; -Ixy Iy -Iyz; -Ixz -Iyz Iz];
% Paralell axis theorem 
% I_CO = I_CG - m*Smtrx(r_bg)^2;
I_CO = diag([Ix Iy Iz]);


% Find M_RB for DOF 1,2 and 6
% Eq 3.49 p.64 in Fossen
M_RB = [    m*eye(3)    -m*Smtrx(r_bg);
         m*Smtrx(r_bg)        I_CO ];
% Extracting DOF 1,2 and 6 from M_RB
M_RB_DOF = [M_RB(1,1) M_RB(1,2) M_RB(1,6);
            M_RB(2,1) M_RB(2,2) M_RB(2,6);
            M_RB(6,1) M_RB(6,2) M_RB(6,6)]


% Find C_RB for DOF 1,2 and 6
% Eq 3.63 p.68 in Fossen
C_RB = [    m*Smtrx(nu2)               -m*Smtrx(nu2)*Smtrx(r_bg);
        m*Smtrx(r_bg)*Smtrx(nu2)            -Smtrx(I_CO*nu2)];

% Extracting DOF 1,2 and 6 from C_RB
C_RB_lin_DOF = [C_RB(1,1) C_RB(1,2) C_RB(1,6);
                C_RB(2,1) C_RB(2,2) C_RB(2,6);
                C_RB(6,1) C_RB(6,2) C_RB(6,6)]

%% Problem 1d
% Check if C = -C' Skew-symmetric property   
if isequal(C_RB_lin_DOF,-C_RB_lin_DOF.') ~= 1
    disp("Not Skew-symmetric")
end

%% Problem 1e



