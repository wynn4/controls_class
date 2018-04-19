function out = ballbeam_SSctrl(in,P)

z_d = in(1);
z = in(2);
theta = in(3);
t = in(4);

% %use digital differentiator to find zdot and thetadot
% persistent zdot
% persistent z_d1
% persistent thetadot
% persistent theta_d1
% % persistent integrator
% % persistent error_dl
% 
% %reset persistent variables at simulation start
% if t < P.Ts
%     zdot = 0;
%     z_d1 = 0;
%     thetadot = 0;
%     theta_d1 = 0;
% %     integrator = 0;
% %     error_dl = 0;
% end

% %compute the digital derivatives
% zdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*zdot...
%    + 2/(2*P.tau+P.Ts)*(z-z_d1);
% thetadot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*thetadot...
% + 2/(2*P.tau+P.Ts)*(theta-theta_d1);
% z_d1 = z;
% theta_d1 = theta;

% %Compute the integrator
% error = z_d - z;
% 
% if abs(zdot)<0.05,
%     integrator = integrator + (P.Ts/2)*(error + error_dl);
% end
% 
% error_dl = error;

%Compute Equilibrium
ye = P.L/2;
Fe = P.m2*P.g/2; %-+ P.m1*P.g*z/P.L;
x_e = [0; ye; 0; 0];

%Implement observer
persistent xhat
persistent dhat
persistent F
if t < P.Ts,
    xhat = [0; 0; 0; 0];
    F = 0;
    dhat = 0;
end
N = 10;
for i=1:N,
    xhat = xhat + ...
        P.Ts/N*(P.A*(xhat - x_e) + P.B*(F - Fe +dhat) + P.L_obs*([z;theta] - P.C*xhat));
    dhat = dhat + P.Ts/N*P.Ld*([z;theta]-P.C*xhat);
end

%Integrator
error = z_d - z;
persistent integrator
persistent error_dl
if t<P.Ts==1,
    integrator = 0;
    error_dl = 0;
end
if abs(xhat(4))<0.1,
    integrator = integrator + (P.Ts/2)*(error+error_dl);
end
error_dl = error;
%Construct the State
%x = [z; zdot; theta; thetadot;];


%Compute the State Feedback Controller
F_tilde = -P.K*(xhat - x_e) + P.kr*(z_d + ye) + P.Ki*integrator - dhat;

%Total Force
F = sat(Fe + F_tilde, P.Fmax);


%Integrator anti-windup
if P.Ki ~= 0,
    F_unsat = Fe + F_tilde;
    integrator = integrator + P.Ts/P.Ki*(F - F_unsat);
    
    out = [F; xhat];
end

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end