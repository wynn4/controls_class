function output = Planar_VTOL_SSctrl(in,P)
    h_d   = in(1);
    z_d   = in(2);
    z     = in(3);
    h     = in(4);
    theta = in(5);
    t     = in(6);

%%%%%%%%%%%%______Observer on h_______%%%%%%%%%%%%%
%equilibrium
Fe = (P.mc+2*P.mr)*P.g;
persistent xhat_h
persistent F
persistent dhat_h
if t<P.Ts,
    xhat_h = [0; 0];
    F      = 0;
    dhat_h = 0;
end
N = 10;
for i=1:N,
    xhat_h = xhat_h + P.Ts/N*...
        (P.A_h*(xhat_h) + P.B_h*(F + dhat_h - Fe) + P.L_h*(h-P.C_h*xhat_h));
dhat_h = dhat_h + P.Ts/N*P.Ld_h*(h-P.C_h*xhat_h);
end

% integrator on h
    error_h = h_d - h;
    persistent integrator_h
    persistent error_d1_h
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator_h  = 0;
        error_d1_h   = 0;
    end
    if abs(xhat_h(2))<0.1,  % note that x3=zdot
        integrator_h = integrator_h + (P.Ts/2)*(error_h+error_d1_h);
    end
    error_d1_h = error_h;

%%%%%%%%%%%________Observer on z_________%%%%%%%%%%%%%%
persistent xhat_z
persistent T
persistent dhat_z
if t<P.Ts,
    xhat_z = [0; 0; 0; 0];
    T      = 0;
    dhat_z = 0;
end
for i=1:N,
    xhat_z = xhat_z + P.Ts/N*...
        (P.A*(xhat_z) + P.B*(T+dhat_z) + P.L_z*([z;theta]-P.C*xhat_z));
    dhat_z = dhat_z + P.Ts/N*P.Ld_z*([z;theta]-P.C*xhat_z);
end
% integrator on z
    error = z_d - z;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator  = 0;
        error_d1    = 0;
    end
    if abs(xhat_z(3))<0.1,  % note that x3=zdot
        integrator = integrator + (P.Ts/2)*(error+error_d1);
    end
    error_d1 = error;

%Compute the state feedback controller

F_tilde = -P.K_h*xhat_h + P.kr_h*h_d + P.Ki_h*integrator_h - dhat_h;
F = sat(Fe + F_tilde, 2*P.fmax);


T_tilde = -P.K*xhat_z + P.kr*z_d + P.Ki_z*integrator - dhat_z;
T = sat(T_tilde, P.taumax);
%xhat = [0 0 0 0 0 0]';
output = [F; T; xhat_h; xhat_z];

%integrator on h anti-windup
if P.Ki_h ~= 0,
    F_unsat = P.Fe + F_tilde;
    integrator_h = integrator_h + P.Ts/P.Ki_h*(F - F_unsat);
end

%integrator on z anti-windup
if P.Ki_z ~= 0,
    T_unsat = T_tilde;
    integrator = integrator + P.Ts/P.Ki_z*(T - T_unsat);
end

end


% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
