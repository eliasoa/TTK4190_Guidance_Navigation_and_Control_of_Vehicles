function xdot = ship(x,u)
% xdot = ship(x,u) returns the time derivative of the state vector: 
% x = [ u v r x y psi delta n ]' for a ship with L = 161 m where:
%
% u     = surge velocity, must be positive  (m/s)    
% v     = sway velocity                     (m/s)
% r     = yaw velocity                      (rad/s)
% x     = position in x-direction           (m)
% y     = position in y-direction           (m)
% psi   = yaw angle                         (rad)
% delta = actual rudder angle               (rad)
% n     = actual shaft velocity             (rps)
% 
% The input vector is :
%
% u       = [ delta_c  n_c ]'  where
%
% delta_c = commanded rudder angle          (rad)
% n_c     = commanded shaft velocity        (rpm)
%
% Author:    Elias Olsen Almenningen
% Date:      10.10.23

% Check of input and state dimensions
if (length(x)~= 8),error('x-vector must have dimension 8 !');end
if (length(u)~= 2),error('u-vector must have dimension 2 !');end

% Dimensional states and input
delta_c = u(1); 
n_c     = u(2);

nu    = x(1:3);         
eta   = x(4:6);         
delta = x(7);
n     = x(8); 

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

% rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MA = -[ Xudot   0     0;
          0   Yvdot Yrdot;
          0   Nvdot Nrdot];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% rigid-body mass matrix
MRB = [ m   0      0 ;
        0   m    m*xg;
        0   m*xg  Iz ];
M = MRB + MA;
Minv = inv(M);

% input matrix
t_thr = 0.05;           % thrust deduction number
X_delta2 = 0;         % rudder coefficients (Section 9.5)
Y_delta = 0;      
N_delta = -1e7;
Bi = [ (1-t_thr) X_delta2;
        0        Y_delta;
        0        N_delta  ];
    
% state-dependent time-varying matrices
CRB = m * nu(3) * [ 0 -1 -xg; 
                    1  0  0; 
                    xg 0  0  ];
                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nu = u v w
% eta = p q r
CA = [             0                        0       Yvdot*x(2)+Yrdot*x(3);
                   0                        0       -Xudot*x(1);
       -Yvdot*x(2)-Yrdot*x(3)        Xudot*x(1)           0  ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C = CRB + CA;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T1 = 20;
T2 = T1;
T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
D = - [ Xu  0   0;
        0   Yv  0;
        0   0  Nr];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = 0.1;                    % form factor giving viscous correction
CR = 0;                     % Residual friction
epsilon = 0.001;            % a small number to ensure that Cf is well defined for u_r=0
S = 2*L*T + 2*B*T + L*B;    % Surface wetted area
Lpp = L;                    % Studass sa det
Rn = Lpp/(1e-6)*abs(x(1));  % Reynolds number
CF = 0.075/((log10(Rn) - 2)^2 + epsilon);   % Friction koefficient
Cf = CF+CR;           
X = -1/2*rho*S*(1+k)*Cf*abs(x(1))*x(1); % ITTC ressistance in surge

Cd_2D = Hoerner(B,T);
% Strip theory: cross−flow drag integrals
dx = L/10; % 10 strips
Ycf = 0;
Ncf = 0;
for xL = -L/2:dx:L/2
    Ucf = abs(x(2) + xL * x(3)) * (x(2) + xL * x(3));
    Ycf = Ycf - 0.5 * rho * T * Cd_2D * Ucf * dx;       % sway force
    Ncf = Ncf - 0.5 * rho * T * Cd_2D * xL * Ucf * dx;  % yaw moment
end 

d = -[X;Ycf;Ncf];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = Rzyx(0,0,eta(3));

% thrust 
thr = rho * Dia^4 * KT * abs(n) * n;    % thrust command (N)

% ship dynamics
u = [ thr delta ]';
tau = Bi * u;
N = C + D;
nu_dot = Minv * (tau - N * nu - d); 
eta_dot = R * nu;    

% Rudder saturation and dynamics (Sections 9.5.2)
if abs(delta_c) >= delta_max
    delta_c = sign(delta_c)*delta_max;
end

delta_dot = delta_c - delta;
if abs(delta_dot) >= Ddelta_max
    delta_dot = sign(delta_dot)*Ddelta_max;
end    

% propeller dynamics
n_dot = (1/10) * (n_c - n);

xdot = [nu_dot' eta_dot' delta_dot n_dot]';

end