function drawRamp(u)

    % process inputs to function
    z = u(1);
    theta    = u(2);
    
    t        = u(3);
    
    L = .5;
    w = .25;
    r = .05;
    
    % define persistent variables 
    persistent ramp_handle
    persistent ball_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plot([-1*L,4*L],[0,0],'k--'); % plot track
        hold on
        ramp_handle = drawRamp_(theta, L, w, []);
        ball_handle = drawBall_(theta, z, r, w, []);
        axis([-1*L, 2*L, -1*L, 2*L]);
        
    
        
    % at every other time step, redraw base and rod
    else 
        drawRamp_(theta, L, w, ramp_handle);
        drawBall_(theta, z, r, w, ball_handle);
    end
end

   
%
%=======================================================================
% drawRamp_
% draw the Ramp
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRamp_(theta, L, w, handle)
  
  pts = [...
      0, -w/16;...
      L, -w/16;...
      L, w/16;...
      0, w/16;...
      ]';
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%=======================================================================
% drawBall_
% draw the Ball
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBall_(theta, z, r, w, handle)
 
  pts = [...
      r*cos(0), r*sin(0)+(r+(w/16));...
      r*cos(pi/6), r*sin(pi/6)+(r+(w/16));...
      r*cos(pi/3), r*sin(pi/3)+(r+(w/16));...
      r*cos(pi/2), r*sin(pi/2)+(r+(w/16));...
      r*cos(2*pi/3), r*sin(2*pi/3)+(r+(w/16));...
      r*cos(5*pi/6), r*sin(5*pi/6)+(r+(w/16));...
      r*cos(pi), r*sin(pi)+(r+(w/16));...      
      r*cos(7*pi/6), r*sin(7*pi/6)+(r+(w/16));...
      r*cos(8*pi/6), r*sin(8*pi/6)+(r+(w/16));...
      r*cos(9*pi/6), r*sin(9*pi/6)+(r+(w/16));...
      r*cos(10*pi/6), r*sin(10*pi/6)+(r+(w/16));...
      r*cos(11*pi/6), r*sin(11*pi/6)+(r+(w/16));...
      r*cos(0), r*sin(0)+(r+(w/16));...

      ]';
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  
  X = pts(1,:);
  Y = pts(2,:);
  X = X + z*cos(theta);
  Y = Y + z*sin(theta);
  
  if isempty(handle),
    handle = fill(X,Y,'r');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end