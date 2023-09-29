function [phi_c,chi_int] = courseHold(chi,chi_c,Vg,chi_int,h)
w_n_phi = 0.5987;                   % natural freq phi, aquired from roll controller
g = 9.81;                           % Gravity

% Tunable parameters
W_chi = 10;                         % Bandwith separator, tuning parameter
zeta_chi = .7;                      % Damping factor, tuning parameter

% Calculating gains
w_n_chi = 1 / W_chi * w_n_phi;              % natural freq chi, Lecture notes
K_p_chi = 2 * zeta_chi * w_n_chi * Vg / g;  % Kp gain, Lecture notes
K_i_chi = w_n_chi^2 * Vg / g;               % Ki gain, Lecture notes

% Control law
e_chi = ssa(chi_c - chi);                       % Error signal
phi_c = e_chi * K_p_chi + chi_int * K_i_chi;    % Control law

% Euler's method: chi_int[k+1]
chi_int = chi_int + h * ssa(chi_c - chi);
end
