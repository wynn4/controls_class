function out=pendulum_ctrl(in,P)
    z_d   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % implement observer
    persistent xhat       % estimated state (for observer)
    persistent F
    if t<P.Ts,
        xhat = [0;0;0;0];
        F    = 0;
    end
    N = 10;
    for i=1:N,
        xhat = xhat + ...
            P.Ts/N*(P.A*xhat+P.B*F+ P.L*([z;theta]-P.C*xhat));
    end

    % integrator
    error = z_d - z;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator  = 0;
        error_d1    = 0;
    end
    if abs(xhat(3))<0.1,  % note that x3=zdot
        integrator = integrator + (P.Ts/2)*(error+error_d1);
    end
    error_d1 = error;

    % compute the state feedback controller
    F_unsat = -P.K*xhat + P.kr*z_d + P.ki*integrator;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki~=0,
       integrator = integrator + P.Ts/P.ki*(F-F_unsat);
    end
    
    out = [F; xhat];

end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end