% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           Elias Olsen Almenningen
% Study program:    MITK
close all;
clear;
% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v2'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.1;    % sampling time [s]
Ns = 10000;    % no. of samples

% Parameters used in simulation
L_oa = 161;                         % Lenght of ship
rho_a = 1.247;                      % density of air

psi_ref = 10 * pi/180;  % desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

% initial states
eta_0 = [0 0 0]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
x = [nu_0' eta_0' delta_0 n_0]';
xd = [0 0 0]';
e_int = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);       % table of simulation data
wait_bar = waitbar(0, 'Starting');
for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1a) Add current disturbance here 
    beta_vs = deg2rad(45);              % current direction in rad
    Vc = 1;                             % Current speed m/s
    uc = Vc*cos(beta_vs - x(6));        %
    vc = Vc*sin(beta_vs - x(6));        %
    nu_c = [ uc vc 0 ]';                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1c) Add wind disturbance here 
    Vw = 10;                            % wind speed m/s
    beta_Vw = deg2rad(135);             % wind direction
    gamma_w = x(6) - beta_Vw - pi;      % wind angle of attack

    u_w = Vw*cos(beta_Vw - x(6));       % wind x speed
    v_w = Vw*sin(beta_Vw - x(6));       % wind y speed
    u_rw = x(1) - u_w;                  % relatice x speed
    v_rw = x(2) - v_w;                  % relatice y speed
    gamma_rw = -atan2(v_rw,u_rw);        % relatice angle of attack angle
    
    cy = 0.95;                          % wind coefficients
    cn = 0.15;                          % wind coefficients
    ALw = 10*L_oa;                      % latteral projected area
    
    C_Y = cy * sin(gamma_rw);           % wind coefficient 
    C_N = cn * sin(2*gamma_rw);         % wind coefficient
    
    V_rw = sqrt(u_rw^2 + v_rw^2);       % relative wind speed
    if t > 200
        Ywind = 0.5*rho_a*V_rw^2*C_Y*ALw;      %
        Nwind = 0.5*rho_a*V_rw^2*C_N*ALw*L_oa; % 
    else
        Ywind = 0;
        Nwind = 0;
    end
    tau_wind = [0 Ywind Nwind]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add a reference model here 
    % if t > 500
    %     psi_ref = -20 * pi/180;  % desired yaw angle (rad)
    % end
    
    xd_dot      = ref_model(xd,psi_ref);
  
    psi_d       = xd(1);
    r_d         = xd(2);
    
    % psi_d       = psi_ref;
    % r_d         = 0;
    
    u_d         = U_ref;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add the heading controller here 
    e_psi       = ssa(x(6) - psi_d);
    
    e_r         = x(3) - r_d;
    
    e_int       = e_int + h * e_psi;
    
    % rudder angle command (rad)
    delta_c     = PID_heading(e_psi,e_r,e_int);  
    % delta_c     = 0.1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1e) Add open loop speed control here
    
    n_c = open_loop_speed_control(U_ref);
    % n_c = 10;                   % propeller speed (rps)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1f) Replace the open loop speed controller, 
    % with a closed loop speed controller here 
    % Define it as a function
    %
    % The result should look like this:
    % n_c = closed_loop_speed_control(u_d,e_u,e_int_u);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ship dynamics
    u = [delta_c n_c]';
    [xdot,u] = ship(x,u,nu_c,tau_wind,h);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d];     
 
    % Euler integration
    x = euler2(xdot,x,h);
    xd = euler2(xd_dot, xd, h);

    waitbar(i/(Ns+1), wait_bar, sprintf('Progress: %d %%', floor(i/(Ns+1)*100)));
end
close(wait_bar);
simdata = simdata(1:i,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
u       = simdata(:,2);                 % m/s
v       = simdata(:,3);                 % m/s
r       = (180/pi) * simdata(:,4);      % deg/s
x       = simdata(:,5);                 % m
y       = simdata(:,6);                 % m
psi     = simdata(:,7);                 % rad
psi_deg = (180/pi) * psi;               % deg
delta_0 = (180/pi) * simdata(:,8);      % deg
n_0     = 60 * simdata(:,9);            % rpm
delta_c = (180/pi) * simdata(:,10);     % deg
n_c     = 60 * simdata(:,11);           % rpm
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s

figure(2)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');

figure(3)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
subplot(312)
plot(t,n_0,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
subplot(313)
plot(t,delta_0,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');

% plotCrab(t,v,u,psi_deg);


%% Create objects for 3-D visualization 
% Since we only simulate 3-DOF we need to construct zero arrays for the 
% excluded dimensions, including height, roll and pitch
z = zeros(length(x),1);
phi = zeros(length(psi),1);
theta = zeros(length(psi),1);

% create object 1: ship (ship1.mat)
new_object('flypath3d_v2/ship1.mat',[x,y,z,phi,theta,psi],...
'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);

% Plot trajectories 
flypath('flypath3d_v2/ship1.mat',...
'animate','on','step',500,...
'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
'font','Georgia','fontsize',12,...
'view',[-25 35],'window',[900 900],...
'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 