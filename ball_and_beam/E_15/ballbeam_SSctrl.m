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

%reset persistent variables at simulation start
if t < P.Ts
    zdot = 0;
    z_d1 = 0;
    thetadot = 0;
    theta_d1 = 0;
end

%compute the digital derivatives
zdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*zdot...
   + 2/(2*P.tau+P.Ts)*(z-z_d1);
thetadot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*thetadot...
+ 2/(2*P.tau+P.Ts)*(theta-theta_d1);
z_d1 = z;
theta_d1 = theta;

%Construct the State
x = [z; zdot; theta; thetadot;];

Fe = P.m2*P.g/2; %+ P.m1*P.g*z/P.L;
%Compute the State Feedback Controller
F = sat(-P.K*x + P.kr*z_d + Fe, P.Fmax);
end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end