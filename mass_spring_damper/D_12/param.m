% system parameters
AP.m = 5*95;  % kg
AP.k = 3*.95; %Kg/s^2 m
AP.b = 0.5*.95; % kg/s
%AP.g = 9.8; % m/s^2


% initial conditions
AP.Z0 = 0;
AP.Zdot0 = 0;
P.Ts = 0.01;
P.Fe = AP.Z0*AP.k;

%PD Gains
AP.kd = 6.57;
AP.kp = 2;
AP.ki = 0.01;

%Saturation

AP.F_max = 15;
