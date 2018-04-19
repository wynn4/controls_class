clear all

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
P.mc = 1*AP.mc;  % kg
P.mr = 1*AP.mr;     % kg
P.ml = P.mr; % kg
P.Jc = 1*AP.Jc; %kg m^2
P.d = 1*AP.d; % m
P.mu = 1*AP.mu; % kg/s
P.g = 9.81; % m/s^2

% saturation limits for each rotor
P.fmax = 10;

% sample rate for controller
P.Ts = 0.01;

% dirty derivative gain for differentiator
P.tau = 0.05;

% mixing matrix
P.mixing = inv([1, 1; P.d, -P.d]);

% equilibrium force and constraints
P.Fe = (P.mc+2*P.mr)*P.g;
P.Ftildemax = 2*P.fmax - P.Fe;
P.taumax = (P.fmax-P.Fe/2)/P.d;

% design equations for longitudinal control
A_h    = 10;
zeta_h = 0.707;
P.kp_h = P.Ftildemax/A_h;
wn_h   = sqrt(P.kp_h/(P.mc+2*P.mr));
P.kd_h = 2*zeta_h*wn_h*(P.mc+2*P.mr);

P.ki_h = 0.5;

% design equations for lateral control
% PD design for lateral inner loop
b0       = 1/(P.Jc+2*P.mr*P.d^2);
A_th     = 1;
zeta_th  = 0.707;
P.kp_th  = P.taumax/A_th;
wn_th    = sqrt(b0*P.kp_th);
P.kd_th  = 2*zeta_th*wn_th/b0;

% DC gain for lateral inner loop
k_DC_th = 1;

%PD design for lateral outer loop
b1       = -P.Fe/(P.mc+2*P.mr);
a1       = P.mu/(P.mc+2*P.mr);
A_z      = 10;
zeta_z   = 0.707;
P.kp_z   = -A_th/A_z;
wn_z     = sqrt(b1*P.kp_z);
P.kd_z   = (2*zeta_z*wn_z-a1)/b1;

P.ki_z   = 0;

%%%%%%%%%%%%%%%%%%%%%__State Space Controller Params__%%%%%%%%%%%%%%%%%%%%

%Desired performance characteristics
wn_h   = 1*sqrt(P.kp_h/(P.mc+2*P.mr));
wn_th    = 1*sqrt(b0*P.kp_th);
wn_z     = 1*sqrt(b1*P.kp_z);

zeta_h = 1*0.707;
zeta_th = 1*0.707;
zeta_z = 1*0.707;

char_poly_h = [1, 2*zeta_h*wn_h, wn_h^2];
des_poles_h = roots(char_poly_h);

char_poly_z_th = conv([1, 2*zeta_z*wn_z, wn_z^2],...
    [1, 2*zeta_th*wn_th, wn_th^2]);
des_poles_z_th = roots(char_poly_z_th);
%State space matrices

%%%%%%%%%%Up and Down%%%%%%%%%%%%%
A_h = [0 1;0 0];
B_h = [0;(1/(P.mc+2*P.mr))];
C_h = [1 0];
D_h = [0];

%Check to see if system is controlable
if rank(ctrb(A_h,B_h)) ~= 2, disp('System Not Controllable');end
P.K_h = place(A_h,B_h,des_poles_h);
P.kr_h = -1/(C_h(1,:)*inv(A_h-B_h*P.K_h)*B_h);


%%%%%%%%%%%Side to Side%%%%%%%%%%%%%%%%
A = [0 0 1 0;0 0 0 1;0 -P.Fe/(P.mc+2*P.mr) P.mu/(P.mc+2*P.mr) 0;...
    0 0 0 0];
B = [0; 0; 0; 1/(P.Jc+2*P.mr*P.d^2)];
C = [1 0 0 0;0 1 0 0];
D = [0; 0];

%Check to see if system is controlable
if rank(ctrb(A,B)) ~= 4, disp('System Not Controllable');end
P.K = place(A,B,des_poles_z_th)
P.kr = -1/(C(1,:)*inv(A-B*P.K)*B)

