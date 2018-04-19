% actual system parameters
AP.mc = 1.0;  % kg
AP.mr = 0.25;     % kg
AP.ml = AP.mr; % kg
AP.Jc = 0.0042; %kg m^2
AP.d = 0.3; % m
AP.mu = 0.1; % kg/s
AP.g = 9.81; % m/s^2

% initial conditions
AP.z0 = 0;
AP.zdot0 = 0;
AP.h0 = 0;
AP.hdot0 = 0;
AP.theta0 = 0;
AP.thetadot0 = 0;

% assumed system paramters used for design
P.mc = 1.0;  % kg
P.mr = 0.25;     % kg
P.ml = AP.mr; % kg
P.Jc = 0.0042; %kg m^2
P.d = 0.3; % m
P.mu = 0.1; % kg/s
P.g = 9.81; % m/s^2


% mixing matrix
P.mixing = inv([1, 1; P.d, -P.d]);

% design equations for longitudinal control
wn_h = 2.2/8;
th_h = 180/pi*asin(sqrt(log(100/15)^2/(log(100/15)^2+pi^2)));
sig_h = 1/20*log(141/1);

Delta_cl_d = poly([-0.2475+j*0.1191,-0.2475-j*0.1191]);

% PD gains
P.kp_h = Delta_cl_d(3)*(P.mc+2*P.mr);
P.kd_h = Delta_cl_d(2)*(P.mc+2*P.mr);
P.F0 = (AP.mc+2*AP.mr)*AP.g;

% design equations for lateral control
% PD design for lateral inner loop
b0       = 1/(P.Jc+2*P.mr*P.d^2);
tr_th    = .8;
wn_th    = 2.2/tr_th;
P.kp_th  = wn_th^2/b0;
P.kd_th  = 2*zeta_th*wn_th/b0;

% DC gain for lateral inner loop
k_DC_th = 1;

%PD design for lateral outer loop
b1       = -P.F0/(P.mc+2*P.mr);
a1       = P.mu/(P.mc+2*P.mr);
tr_z     = 10*tr_th;
zeta_z   = 0.707;
wn_z     = 2.2/tr_z;
P.kp_z   = wn_z^2/b1;
P.kd_z   = (2*zeta_z*wn_z-a1)/b1;



