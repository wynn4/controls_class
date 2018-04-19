function u=VTOL_ctrl(in,P)
    h_d   = in(1);
    z_d   = in(2);
    z     = in(3);
    h     = in(4);
    theta = in(5);
    t     = in(6);
    
    % set persistent flag to initialize integrators and differentiators at
    % the start of the simulation
    persistent flag
    if t<P.Ts,
        flag = 1;
    else
        flag = 0;
    end
    
    % longitudinal control for alitutde
    % equilibrium force
    F_tilde = PID_h(h_d,h,flag,P.kp_h,P.ki_h,P.kd_h,P.Ftildemax,P.Ts,P.tau);
    F = P.Fe + F_tilde;
    
    % lateral control for position
    % outer-loop: compute the desired pitch angle using position error
    theta_d = PID_z(z_d,z,flag,P.kp_z,P.ki_z,P.kd_z,30*pi/180,P.Ts,P.tau);
    % inner-loop: compute the torque using pitch error
    tau = PD_th(theta_d,theta,flag,P.kp_th,P.kd_th,P.taumax,P.Ts,P.tau);

    u = [F; tau];
end

%------------------------------------------------------------
% PID control for altitude
function u = PID_h(h_d,h,flag,kp,ki,kd,limit,Ts,tau)
    % declare persistent variables
    persistent integrator
    persistent hdot
    persistent error_d1
    persistent h_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        hdot    = 0;
        error_d1    = 0;
        h_d1    = 0;
    end
    
    % compute the error
    error = h_d-h;
    % update derivative of h
    hdot = (2*tau-Ts)/(2*tau+Ts)*hdot + 2/(2*tau+Ts)*(h-h_d1);
    % update integral of error
    if abs(hdot)<.005,
        integrator = integrator + (Ts/2)*(error+error_d1);
    end
    % update delayed variables for next time through the loop
    error_d1 = error;
    h_d1     = h;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*hdot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    %if ki~=0,
    %    integrator = integrator + Ts/ki*(u-u_unsat);
    %end
end


%------------------------------------------------------------
% PID control for position
function u = PID_z(z_c,z,flag,kp,ki,kd,limit,Ts,tau)
    % declare persistent variables
    persistent integrator
    persistent zdot
    persistent error_d1
    persistent z_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        zdot    = 0;
        error_d1    = 0;
        z_d1    = 0;
    end
    
    % compute the error
    error = z_c-z;
    % update derivative of z
    zdot = (2*tau-Ts)/(2*tau+Ts)*zdot + 2/(2*tau+Ts)*(z-z_d1);
    % update integral of error
    if abs(zdot)<.005,
        integrator = integrator + (Ts/2)*(error+error_d1);
    end
    % update delayed variables for next time through the loop
    error_d1 = error;
    z_d1     = z;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*zdot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    %if ki~=0,
    %    integrator = integrator + Ts/ki*(u-u_unsat);
    %end
end


%------------------------------------------------------------
% PD control for angle theta
function u = PD_th(theta_c,theta,flag,kp,kd,limit,Ts,tau)
    % declare persistent variables
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if flag==1,
        thetadot    = 0;
        theta_d1    = 0;
    end
    
    % compute the error
    error = theta_c-theta;
    % update derivative of y
    thetadot = (2*tau-Ts)/(2*tau+Ts)*thetadot + 2/(2*tau+Ts)*(theta-theta_d1);
    % update delayed variables for next time through the loop
    theta_d1 = theta;

    % compute the pid control signal
    u_unsat = kp*error - kd*thetadot;
    u = sat(u_unsat,limit);
    
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end