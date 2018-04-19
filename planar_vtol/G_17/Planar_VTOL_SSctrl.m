function output = Planar_VTOL_SSctrl(in,P)
    h_d   = in(1);
    z_d   = in(2);
    z     = in(3);
    h     = in(4);
    theta = in(5);
    t     = in(6);


%Digitally compute the derivatives zdot and thetadot
persistent zdot
persistent z_dl
persistent thetadot
persistent theta_dl
persistent hdot
persistent h_dl
persistent integrator_h
persistent h_error_dl
persistent integrator_z
persistent z_error_dl

%Initialize persistent variables at simulation start
if t<P.Ts,
    zdot = 0;
    z_dl = 0;
    thetadot = 0;
    theta_dl = 0;
    hdot = 0;
    h_dl = 0;
    integrator_h = 0;
    h_error_dl = 0;
    integrator_z = 0;
    z_error_dl = 0;
end

%Digital differentiator
zdot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*zdot...
    + 2/(2*P.tau + P.Ts)*(z - z_dl);

thetadot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*thetadot...
    + 2/(2*P.tau + P.Ts)*(theta - theta_dl);

hdot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*hdot...
    + 2/(2*P.tau + P.Ts)*(h - h_dl);

z_dl = z;
theta_dl = theta;
h_dl = h;

%Implement the Integrator on h

error_h = h_d - h;
if abs(hdot)<0.1,
    integrator_h = integrator_h + (P.Ts/2)*(error_h + h_error_dl);
end
h_error_dl = error_h;

%Implement the Integrator on z

error_z = z_d - h;
if abs(zdot)<0.05,
    integrator_z = integrator_z + (P.Ts/2)*(error_z + z_error_dl);
end
z_error_dl = error_z;


%Construct the states
x_h = [h; hdot];

x = [z; theta; zdot; thetadot];

%Compute the state feedback controller

F_tilde = -P.K_h*x_h + P.kr_h*h_d + P.Ki_h*integrator_h;
F = sat(P.Fe + F_tilde, 2*P.fmax);
T_tilde = -P.K*x + P.kr*z_d + P.Ki_z*integrator_z;
T = sat(T_tilde, P.taumax);

output = [F; T];

%integrator on h anti-windup
if P.Ki_h ~= 0,
    F_unsat = P.Fe + F_tilde;
    integrator_h = integrator_h + P.Ts/P.Ki_h*(F - F_unsat);
end

%integrator on z anti-windup
if P.Ki_z ~= 0,
    T_unsat = T_tilde;
    integrator_z = integrator_z + P.Ts/P.Ki_z*(T - T_unsat);
end

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
