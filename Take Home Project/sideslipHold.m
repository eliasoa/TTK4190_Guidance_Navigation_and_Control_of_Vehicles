function [delta_r_c, beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h)
D2R = pi / 180;             % degrees to radians
delta_r_max = 30 * D2R;     % maximum rudder [rad]

% Tunable parameters 
e_beta_max = 20 * D2R;      % max error, tuning parameter
zeta_beta = 1;              % damping factor, tuning paramerter

% Calculating gains
K_p_beta = delta_r_max / e_beta_max ;                           % Kp gain, Lecture notes
w_n_beta = ( a_beta1 + a_beta2 * K_p_beta) / (2 * zeta_beta);   % natural freq beta
K_i_beta = w_n_beta^2 / a_beta2;                                % Ki gain, Lecture notes

% Control law
error_beta = 0 - beta;  % Error in sideslip
delta_r_c = error_beta * K_p_beta - K_i_beta * beta_int;

% Euler's method: beta_int[k+1]
beta_int = beta_int + h * beta;
end