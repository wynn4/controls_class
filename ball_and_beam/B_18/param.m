% inverted Pendulum parameter file
clear all

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
P.m1 = AP.m1;%*0.95;  % kg
P.m2 = AP.m2;%*1.05;     % kg
P.ell = AP.ell;%*0.98; % m
P.b = AP.b;%*1.05; % N m
P.g = 9.8; % m/s^2

% input constraint
P.F_max = 5;

% sample rate for controller
P.Ts = 0.01;

% gain for dirty derivative
P.tau = 0.05;

% % select PD gains
% A_th = 20*pi/180;
% zeta_th = 0.707;
% P.kp_th = -P.F_max/A_th;
% wn_th = sqrt(-(P.m1+P.m2)*P.g/P.m2/P.ell-P.kp_th/P.m2/P.ell);
% P.kd_th = -2*zeta_th*wn_th*P.m2*P.ell;
% 
% % DC gain for inner loop
% k_DC_th = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);
% 
% % PD design for outer loop
% A_z = 1;
% zeta_z = 0.8;%0.707;
% P.kp_z = A_th/A_z;
% wn_z = sqrt(P.m1*P.g/P.m2*P.kp_z);
% P.kd_z = P.m2/P.m1/P.g*(-P.b/P.m2 + 2*zeta_z*wn_z);

% draw root locus
% L = tf([P.m1*P.g/P.m2],[1,(P.b/P.m2+P.m1*P.g*P.kd_z/P.m2), P.m1*P.g*P.kp_z/P.m2,0]);
% figure(2), clf, rlocus(L);

% select integrator gain
%P.ki_z = 0.0001;

% state space design
P.A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.m1*P.g/P.m2, -P.b/P.m2, 0;...
    0, (P.m1+P.m2)*P.g/P.m2/P.ell, P.b/P.m2/P.ell, 0;...
];
P.B = [0; 0; 1/P.m2; -1/P.m2/P.ell ];
P.C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% form augmented system
Cout = [1,0,0,0];
A1 = [P.A, zeros(4,1); Cout, 0];
B1 = [P.B; 0];

% pick poles
wn_th   = 2;
zeta_th = 0.707;
wn_z    = 1;
zeta_z  = 0.8;
des_char_poly = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                      [1,2*zeta_th*wn_th,wn_th^2]),...
                 poly(-10));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
    P.kr = -1/(Cout*inv(P.A-P.B*P.K)*P.B);
end

% observer design
% pick observer poles
wn_th_obs   = 10*wn_th;
wn_z_obs    = 10*wn_z;
des_obsv_char_poly = conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=4, 
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A', P.C', des_obsv_poles)';
end




