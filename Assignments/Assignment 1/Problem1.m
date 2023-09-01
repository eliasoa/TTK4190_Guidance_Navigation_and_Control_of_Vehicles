%% Parameters
m = 180;
R33 = 2.0;
I_3 = eye(3);
I_g = m*R33^2*I_3;
invI_g = inv(I_g);

%% Problem 1.1

A = [zeros(3,3) 1/2*I_3;
     zeros(3,3) zeros(3,3)];
B = [zeros(3,3) invI_g].';

%% Problem 1.2
k_d = 40;
k_p = 2;
K = [-k_p*eye(3) -k_d*eye(3)];
A_sys = (A+B*K)
eig(A_sys)
