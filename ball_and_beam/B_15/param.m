% inverted Pendulum parameter file

% actual system parameters used in simulation model
AP.m1 = 0.25;  % kg
AP.m2 = 1;     % kg
AP.ell = 0.5; % m
AP.b = 0.05; % N m
AP.g = 9.8; % m/s^2


% initial conditions
AP.z0 = 0;
AP.zdot0 = 0;
AP.theta0 = 0;
AP.thetadot0 = 0;

% system parameters used for design
P.m1 = 0.95*AP.m1;  % kg
P.m2 = 1.05*AP.m2;     % kg
P.ell = 0.98*AP.ell; % m
P.b = 1.05*AP.b; % N m
P.g = 9.8; % m/s^2

% input constraint
P.F_max = 5;

% select PD gains
A_th = 2*pi/180;
zeta_th = 0.9;%0.707;
P.kp_th = -P.F_max/A_th;
wn_th = sqrt(-(P.m1+P.m2)*P.g/P.m2/P.ell-P.kp_th/P.m2/P.ell);
P.kd_th = -2*zeta_th*wn_th*P.m2*P.ell;

% DC gain for inner loop
k_DC_th = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

% PD design for outer loop
A_z = 1;
zeta_z = 0.707;
P.kp_z = A_th/A_z;
wn_z = sqrt(P.m1*P.g/P.m2*P.kp_z);
P.kd_z = P.m2/P.m1/P.g*(-P.b/P.m2 + 2*zeta_z*wn_z);

% draw root locus
L = tf([P.m1*P.g/P.m2],[1,(P.b/P.m2+P.m1*P.g*P.kd_z/P.m2), P.m1*P.g*P.kp_z/P.m2,0]);
figure(2), clf, rlocus(L);

% select integrator gain
P.ki_z = 0.0001;

P.Ts = 0.01;




