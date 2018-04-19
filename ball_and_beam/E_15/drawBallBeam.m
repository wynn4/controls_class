function drawBallBeam(u)

    % process inputs to function
    theta    = u(1);
    y        = u(2);
    t        = u(3);
    
    % drawing parameters
    L = 0.5;
    R = 0.05;

    
    % define persistent variables 
    persistent ball_handle
    persistent beam_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plot([0,L],[0,0],'k'); % plot track
        hold on
        ball_handle = drawBall(y, theta, L, R, []);
        beam_handle = drawBeam(y, theta, L, R, []);
        axis([-L/5, L+L/5, -L, L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawBall(y, theta, L, R, ball_handle);
        drawBeam(y, theta, L, R, beam_handle);
    end
end

   
%
%=======================================================================
% drawBall
% draw the ball
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBall(y, theta, L, R, handle)
  
  N = 20;
  xi = 0:(2*pi/10):2*pi;
  X = y*cos(theta)-R*sin(theta)+R*cos(xi);
  Y = y*sin(theta)+R*cos(theta)+R*sin(xi);
  
  if isempty(handle),
    handle = fill(X,Y,'b');
    %handle = plot(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=======================================================================
% drawBeam
% draw the beam
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBeam(y, theta, L, R, handle)

  
  X = [0, L*cos(theta)];
  Y = [0, L*sin(theta)];

  if isempty(handle),
    handle = plot(X, Y, 'g');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

  