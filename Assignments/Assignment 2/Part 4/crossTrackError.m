function [e_y,pi_p] = crossTrackError(xk1,yk1,xk,yk,x,y)
% Equation 12.54 and 12.55
pi_p = atan2(yk1 - yk,xk1 - xk);

e     = R(pi_p).' * [x - xk; y - yk]; 
e_y   = e(2);
end

function R = R(phi)
    R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
end