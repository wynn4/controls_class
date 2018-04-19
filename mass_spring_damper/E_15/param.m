% actual system parameters
AP.m = 0.5;   % kg
AP.ell = 0.3; % m
AP.b = 0.01;  % N m s
AP.g = 9.8;   % m/s^2

% initial conditions
AP.theta0 = 0;
AP.thetadot0 = 0;
P.Ts = 0.01;

% parameters known to controller
P.m = 0.95*AP.m;   % kg
P.ell = 1.05*AP.ell; % m
P.b = .97*AP.b;  % N m s
P.g = 9.8;   % m/s^2

% equalibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

% saturation constraint
P.tau_max = 1;
tau_max = P.tau_max-P.tau_e;

% select PD gains
A_th = 50*pi/180;
zeta = 0.707;
P.kp = tau_max/A_th;
wn = sqrt(3*P.kp/(P.m*P.ell^2));
P.kd = 2*zeta*wn*(P.m*P.ell^2)/3-P.b;

roots([1,2*zeta*wn,wn^2]);

% draw root locus
L = tf([3/P.m/P.ell^2],[1,(3*P.b+3*P.kd)/P.m/P.ell^2,3*P.kp/P.m/P.ell^2,0]);
figure(2), clf, rlocus(L);

% select integrator gain
P.ki = 0.1;


