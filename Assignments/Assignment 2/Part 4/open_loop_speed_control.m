function n_c = open_loop_speed_control(U_ref)
% Ship variables
m                   = 17.0677e6;        % mass (kg)          
Xudot               = -8.9830e5;        % added mass in surge
T1                  = 20;               % linear damping time constant
Xu                  = -(m-Xudot)/T1;    % linear damping in surge

% Propeller variables
Dia                 = 3.3;              % propeller diameter (m)
rho                 = 1025;             % density of water (m/s^3)
Ja                  = 0;                % advance number
PD                  = 1.5;              % pitch/diameter ratio
AEAO                = 0.65;             % blade area ratio
z                   = 4;                % number of propeller blades
[KT,~]              = wageningen(Ja,PD,AEAO,z);

t                   = 0.05;             % thrust deduction number p. 164 in Fossen

% Combining the equation in task d) and inserting T (eq. 9.7 in Fossen)
n_c_squared         = U_ref * Xu / ((t-1) *rho * Dia^4 * KT);    
n_c                 = sign(n_c_squared) * sqrt(abs(n_c_squared));
end