% actual system parameters
AP.m = 5*95;  % kg
AP.k = 3*.95; %Kg/s^2 m
AP.b = 0.5*.95; % kg/s
%AP.g = 9.8; % m/s^2


% initial conditions
AP.Z0 = 0;
AP.Zdot0 = 0;
P.Ts = 0.01;

%equilibrium and force
AP.F_e = AP.Z0*AP.k;

%PID Gains
AP.kd = 3;
AP.kp = -2.5;
AP.ki = 0.000;

%Saturation

AP.F_max = 15;
F_max = AP.F_max-AP.F_e;
