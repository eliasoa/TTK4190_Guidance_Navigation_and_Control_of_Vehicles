function chi_d = LOS_guidance(e_y,pi_p)
L       = 161;        
delta   = 10*L;        %Sikkert feil
Kp      = 1/delta;
chi_d   = pi_p - atan2(Kp*e_y,1);
end