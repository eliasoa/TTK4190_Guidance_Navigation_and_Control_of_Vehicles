%% Problem 1a
Ja                  = 0;    % advance number
PD                  = 1.5;  % pitch/diameter ratio
AEAO                = 0.65; % blade area ratio
z                   = 4;    % number of propeller blades
[KT,KQ]             = wageningen(Ja,PD,AEAO,z);

%% Problem 1b

Im      = 10^5;                             % (kg m^2)
Km      = 0.6;                              % (s^-1)
Tm      = 10;                               % (s)
tau     = 0;                                % time delay (s)

num = [0 Km];
den = [Tm 1];

e = 0;

[A,B,C,D] = tf2ss(num,den)

edot = A*e + B * Y

e = euler2(edot,e,h)

% Y = Q/Km;
% Qm = 1/Tm * (-Qm + Km * y);
% n_dot = I_m^(-1) * Qm - Q;