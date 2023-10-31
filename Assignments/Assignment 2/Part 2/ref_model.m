function xd_dot = ref_model(xd,psi_ref)
w_ref     = 0.03;
zeta    = 1;

a1 = w_ref +  2*zeta*w_ref;
a2 = 2*zeta*w_ref^2 + w_ref^2;
a3 = w_ref^3;
b3 = a3;
A = [ 0 1 0; 0 0 1; -a3 -a2 -a1];
B = [0 0 b3]';

xd_dot = A*xd + B*psi_ref;
end