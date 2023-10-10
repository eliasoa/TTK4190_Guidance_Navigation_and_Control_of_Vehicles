clear
%% Real Ship
L = 161;                % length (m)
B = 21.8;               % beam (m)
H = 15.8;               % heighy (m)
m = 17.0677e6;          % mass (kg)
rho_m = m/(L*B*H);      % density of ship (kg/m^3)
rho = 1025;             % density of water (m/s^3)
T = 4.74;               % Draft (m)
%% Problem 2a
% Eq. 4.11 p.74 in Fossen
nabla = m/rho;

%% Problem 2b
%p.86 in Fossen
A_wp = nabla/T

%% Problem 2c Finding GM_T and GM_L
% Following example 4.2 in Fossen

KG = 7.9;    % Disance from keel to CG(m)
% Eq. (4.38) in Fossen
KB = 1/3*((5*T)/2 - nabla/A_wp);
BG = KG-KB;

I_T = 1/12*B^3*L;
I_L = 1/12*L^3*B;

BM_T = I_T/nabla;
BM_L = I_L/nabla;

GM_T = BM_T - BG
GM_L = BM_L - BG
