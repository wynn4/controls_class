%Planar Quad Parameters

AP.mc = 1;
AP.Jc = .0042;
AP.mr = .25;
AP.ml = .25;
AP.d = .3;
AP.mu = 0.1;
AP.g = 9.81;

%Initial conditions

AP.theta0 = 0;
AP.thetadot0 = 0;
AP.Z0 = 0;
AP.Zdot0 = 0;
AP.h0 = 0;
AP.hdot0 = 0;

%PD controller gains

AP.kd = .741;
AP.kp = 0.1135;
AP.ki = .001;


