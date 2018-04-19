function F = ballbeam_SSctrl(in,P)

z_d = in(1);
z = in(2);
theta = in(3);
t = in(4);

%use digital differentiator to find zdot and thetadot
persistent zdot
persistent z_d1
persistent thetadot
persistent theta_d1
persistent integrator
persistent error_dl

%reset persistent variables at simulation start
if t < P.Ts
    zdot = 0;
    z_d1 = 0;
    thetadot = 0;
    theta_d1 = 0;
    integrator = 0;
    error_dl = 0;
end

%compute the digital derivatives
zdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*zdot...
   + 2/(2*P.tau+P.Ts)*(z-z_d1);
thetadot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*thetadot...
+ 2/(2*P.tau+P.Ts)*(theta-theta_d1);
z_d1 = z;
theta_d1 = theta;

%Compute the integrator
error = z_d - z;

if abs(zdot)<0.05,
    integrator = integrator + (P.Ts/2)*(error + error_dl);
end

error_dl = error;


%Construct the State
x = [z; zdot; theta; thetadot;];

Fe = P.m2*P.g/2; %+ P.m1*P.g*z/P.L;
%Compute the State Feedback Controller
F_tilde = -P.K*x + P.kr*z_d + P.Ki*integrator;

%Total Force
F = sat(Fe + F_tilde, P.Fmax);
%Integrator anti-windup
if P.Ki ~= 0,
    F_unsat = Fe + F_tilde;
    integrator = integrator + P.Ts/P.Ki*(F - F_unsat);
end

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end