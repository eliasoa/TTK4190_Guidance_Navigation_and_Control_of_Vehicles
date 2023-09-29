function [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)
% [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)

% v_g = ground velocity vector expressed in BODY [m/s]
% v_w = wind velocity vector expressed in BODY [m/s]

% air velocity vector expressed in BODY [m/s]
v_a = v_g - v_w;

Va = norm(v_a); % air speed
Vg = norm(v_g); % ground speed
Vw = norm(v_w); % wind speed


% air velocity vector in BODY in component form
u_r = v_a(1);
v_r = v_a(2);
w_r = v_a(3);

% angle of attack using eq (2.8) in Beard & McLane 
alpha = atand(w_r/u_r);

% side slip angle using eq (2.9) in Beard & McLane
beta = asind(v_r/Va);
end