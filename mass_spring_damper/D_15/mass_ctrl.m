function F=mass_ctrl(in,P,AP)
    Z_c = in(1);
    Z   = in(2);
    t       = in(3);
    
    % set persistent flag to initialize integrator and differentiator at
    % the start of the simulation
    persistent flag
    if t<P.Ts,
        flag = 1;
    else
        flag = 0;
    end
      
    % compute equilibrium force F_e
    F_e = AP.k*AP.Z0;
    % compute the linearized force using PID
    F_tilde = PID_Z(Z_c,Z,flag,AP.kp,AP.ki,AP.kd,AP.F_max,P.Ts,P.F_e);
    % compute total force
    F = F_e + F_tilde;
    
end

%------------------------------------------------------------
% PID control for position Z
function u = PID_Z(Z_c,Z,flag,kp,ki,kd,limit,Ts,F)
    % declare persistent variables
    persistent integrator
    persistent Zdot
    persistent error_d1
    persistent z_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        Zdot    = 0;
        error_d1    = 0;
        z_d1    = 0;
    end
    
    % compute the error
    error = Z_c - Z;
    % update derivative of y
    Zdot = (2*F-Ts)/(2*F+Ts)*Zdot + 2/(2*F+Ts)*(Z-z_d1);
    % update integral of error
    if abs(Zdot)<0.05,
        integrator = integrator + (Ts/2)*(error+error_d1);
    end
    % update delayed variables for next time through the loop
    error_d1 = error;
    z_d1 = Z;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*Zdot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    if ki~=0,
        integrator = integrator + Ts/ki*(u-u_unsat);
    end
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end