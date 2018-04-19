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

%Initialize persistent variables at simulation start
if t<P.Ts,
    zdot = 0;
    z_dl = 0;
    thetadot = 0;
    theta_dl = 0;
    hdot = 0;
    h_dl = 0;
end

zdot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*zdot...
    + 2/(2*P.tau + P.Ts)*(z - z_dl);

thetadot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*thetadot...
    + 2/(2*P.tau + P.Ts)*(theta - theta_dl);

hdot = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)*hdot...
    + 2/(2*P.tau + P.Ts)*(h - h_dl);

z_dl = z;
theta_dl = theta;
h_dl = h;

%Construct the states
x_h = [h; hdot];

x = [z; theta; zdot; thetadot];

%Compute the state feedback controller

F = sat(-P.K_h*x_h + P.kr_h*h_d + P.Fe, 2*P.fmax);
T = sat(-P.K*x + P.kr*z_d, P.taumax);

output = [F; T];

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
