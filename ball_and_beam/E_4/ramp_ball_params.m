function AP=ramp_ball_params()%ramp ball system parameters

AP.m1 = .35; %kg
AP.m2 = 2.0; %kg
AP.l = 0.5; %m
AP.g = 9.8; %m/s^2

%Initial conditions
AP.Z0 = .25;
AP.Zdot0 = 0;
AP.theta0 = 0;
AP.thetadot0 = 0;