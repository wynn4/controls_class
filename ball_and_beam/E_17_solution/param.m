clear all

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
P.m1 = AP.m1*0.95;  % kg
P.m2 = AP.m2*1.05;     % kg
P.L  = AP.L*0.95; % m
P.g  = AP.g; % m/s^2

% sample rate for controller
P.Ts = 0.01;

% parameter for dirty derivative
P.tau = 0.05;

% limits on force
P.Fmax = 15; % N

% % gains for the inner loop
% a1 = P.L/(P.m2*P.L^2/3+P.m1*ye^2);
% A_th = 1*pi/180; % m
% zeta_th = 0.6;
% P.kp_th = (P.Fmax-Fe)/A_th;
% wn_th = sqrt(a1*P.kp_th);
% P.kd_th = 2*zeta_th*wn_th/a1;
% 
% % closed loop poles of the inner loop
% roots([1,2*zeta_th*wn_th,wn_th^2]);
% 
% % gains for outer loop
% A_z = P.L/2;
% P.kp_z = -A_th/A_z;
% wn_z = sqrt(-5/7*P.g*P.kp_z);
% zeta_z = 0.75;
% P.kd_z = -14*zeta_z*wn_z/(5*P.g);
% 
% % closed loop poles of the outer loop
% %roots([1,2*zeta_z*wn_z,wn_z^2]);
% 
% % draw root locus
% %figure(2), clf, rlocus([P.g],[1,-P.g*P.kd_z,-P.g*P.kp_z,0])
% 
% P.ki_z = -0.01;

% state space design
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.m1*P.g/((P.m2*P.L^2)/3+P.m1*(P.L/2)^2), 0, 0;...
    -P.g, 0, 0, 0;...
];
B = [0; 0; P.L/(P.m2*P.L^2/3+P.m1*P.L^2/4); 0; ];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% form augmented system
Cout = [0,1,0,0];
A1 = [A, zeros(4,1); Cout, 0];
B1 = [B; 0];

% gains for pole locations
wn_z    = 2*0.6991;
zeta_z  = 0.707;
wn_th   = 23.0115;
zeta_th = 0.707;

ol_char_poly = charpoly(A);
des_char_poly = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                          [1,2*zeta_th*wn_th,wn_th^2]),...
                      poly(-0.5));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
    P.kr = -1/(Cout*inv(A-B*P.K)*B);
end



