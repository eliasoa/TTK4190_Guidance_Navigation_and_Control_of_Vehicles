%% USER INPUTS
h = 0.1;                     % sample time (s)
N  = 2000;                    % number of samples. Should be adjusted

% model parameters
m = 180;
r = 2;
I = m*r^2*eye(3);            % inertia matrix
I_inv = inv(I);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi = -5*deg2rad;            % initial Euler angles
theta = 10*deg2rad;
psi = -20*deg2rad;

q = euler2q(phi,theta,psi);   % transform initial Euler angles to q

w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,14);        % memory allocation
reference_table = zeros(N+1,3); % memory allocation for reference signal
% Controller
kd = 400;
kp = 20;
Kd = kd*eye(3);


%% FOR-END LOOP
for i = 1:N+1
   t = (i-1)*h;                  % time
   phi_d = 0;   % time-varying reference signal as Euler angles 
   theta_d = 15*cos(0.1*t)*deg2rad;
   psi_d = 10*sin(0.05*t)*deg2rad;

   q_desired = euler2q(phi_d, theta_d, psi_d);   % desired path

   q_desired_conjugate = [q_desired(1); -q_desired(2:4)];   % conjugate quat

   q_tilde = quatprod(q_desired_conjugate,q);
   epsilon_tilde = q_tilde(2:4);

   tau = -Kd*w - kp*epsilon_tilde;         % control law

   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics
   
   table(i,:) = [t q' phi theta psi w' tau'];  % store data in table
   reference_table(i,:) = [phi_d, theta_d, psi_d];
   
   q = q + h*q_dot;	             % Euler integration
   w = w + h*w_dot;
   
   q  = q/norm(q);               % unit quaternion normalization
end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11);  
tau     = table(:,12:14);

phi_d = rad2deg*reference_table(:,1);
theta_d = rad2deg*reference_table(:,2);
psi_d = rad2deg*reference_table(:,3);

figure (1); clf;
subplot(3,1,1);
title('Euler angles');
hold on;
plot(t, phi, 'b');
plot(t, phi_d, 'b--');
hold off;
grid on;
legend('\phi', '\phi_d');


subplot(3,1,2);
hold on;
plot(t, theta, 'r');
plot(t, theta_d, 'r--');
hold off;
grid on;
legend('\theta', '\theta_d');
ylabel('angle [deg]');

subplot(3,1,3);
hold on;
plot(t, psi, 'g');
plot(t, psi_d, 'g--');
hold off;
grid on;
legend('\psi', '\psi_d');
xlabel('time [s]'); 


figure (2); clf;
hold on;
plot(t, w(:,1), 'b');
plot(t, w(:,2), 'r');
plot(t, w(:,3), 'g');
hold off;
grid on;
legend('p', 'q', 'r');
title('Angular velocities');
xlabel('time [s]'); 
ylabel('angular rate [deg/s]');

figure (3); clf;
hold on;
plot(t, tau(:,1), 'b');
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');

