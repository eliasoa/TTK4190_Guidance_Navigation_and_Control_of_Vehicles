function [delta_a_c,phi_int] = rollAttitudeHold(a_phi1,a_phi2,p,phi,phi_c,phi_int,h)
D2R = pi / 180;                                         % degrees to radians
delta_a_max = 21 * D2R;                                 % maximum aileron [rad]
e_phi_max = 15 * D2R;                                   % max error, tuning parameter

% Tunable parameters
K_p_phi = delta_a_max / e_phi_max;                      % Kp gain, Lecture notes
zeta_phi = 1;                                           % damping factor

% Calculating gains
w_n_phi = abs ( sqrt ( K_p_phi * a_phi2 ) );                % natural freq phi
K_d_phi = ( 2 * zeta_phi * w_n_phi - a_phi1 ) / ( a_phi2 ); % Kd gain, Lecture notes
K_i_phi = ( K_p_phi *w_n_phi ) / 10;                        % Ki gain, pole placement from Fossen

% Control law
e_phi = ssa(phi_c - phi);                               % Error signal
% d_a_marked = e_phi * K_p_phi;                         % delta_a_marked PD
d_a_marked = e_phi*K_p_phi + K_i_phi*phi_int;           % delta_a_marked PID

delta_a_c = -d_a_marked - K_d_phi*p;                    % Control law

% Euler's method: phi_int[k+1]
phi_int = phi_int + h * (phi_c - phi);
end
