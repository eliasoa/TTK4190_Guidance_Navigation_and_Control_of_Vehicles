% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

% input matrix
t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0 0; 0 Yvdot Yrdot; 0 Nvdot Nrdot ];

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = inv(MRB + MA);

ud = 7; %

% linearized corriolis matrix
CRB = [0 0 0; 0 0 m*ud; 0 0 m*xg*ud];
CA = [0 0 0; 0 0 -Xudot*ud; 0 ud*(-Yvdot + Xudot) -Yrdot*ud];
C = CA+CRB;

% linear damping
T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
D = -diag([Xu Yv Nr]);

%% Problem 2 b
% Må hente ut sway og yaw 
Minv = Minv(2:3,2:3);
N = C(2:3,2:3) + D(2:3,2:3);
b = 2*ud*[-Y_delta;-N_delta];
% Make the state space
A = Minv*(-N);
B = Minv*b;
C = [0 1];
D_ss = 0;

[num,den] = ss2tf(A,B,C,D_ss);
%% Problem 2 c
num_s = num*(1/den(3));
den_s = den*(1/den(3));

K = num_s(3);
T3 = num_s(2)/K;

root = roots(den_s);
T1 = -1/root(1);
T2 = -1/root(2);

T = T1 + T2 - T3; % 169.5493