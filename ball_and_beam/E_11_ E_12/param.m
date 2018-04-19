% actual system parameters
AP.m1 = 0.35;  % kg
AP.m2 = 2;     % kg
AP.L = 0.5; % m
AP.g = 9.8; % m/s^2


% initial conditions
AP.theta0 = 0;
AP.thetadot0 = 0;
AP.y0 = 0;
AP.ydot0 = 0;

% parameters known to the controller
P.m1 = 0.35;  % kg
P.m2 = 2;     % kg
P.L = 0.5; % m
P.g = 9.8; % m/s^2

% limits on force
P.Fmax = 15; % N
ye = P.L/2;
Fe = P.m1*P.g*ye/P.L+P.m2*P.g/2

% gains for the inner loop
a1 = P.L/(P.m2*P.L^2/3+P.m1*ye^2);
A_th = 1*pi/180; % m
zeta_th = 0.6;
P.kp_th = (P.Fmax-Fe)/A_th
wn_th = sqrt(a1*P.kp_th)
P.kd_th = 2*zeta_th*wn_th/a1

% closed loop poles of the inner loop
roots([1,2*zeta_th*wn_th,wn_th^2])


% gains for outer loop
A_z = P.L/2;
P.kp_z = -A_th/A_z
wn_z = sqrt(-5/7*P.g*P.kp_z)
zeta_z = 0.75;
P.kd_z = -14*zeta_z*wn_z/(5*P.g)

% closed loop poles of the outer loop
roots([1,2*zeta_z*wn_z,wn_z^2])

