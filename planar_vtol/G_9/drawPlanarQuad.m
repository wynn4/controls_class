function drawPlanarQuad(u)

    % process inputs to function
    theta = u(1);
    Z = u(2);
    h = u(3);
    t = u(4);
    
    L = 1;
    %w = 1;
    
    
    % define persistent variables 
    persistent body_handle
    persistent target_handle
    persistent line_handle
       
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        plot([-6*L,6*L],[0,0],'k'); % plot track
        hold on
        body_handle = drawBody_(theta, Z, h, L, []);
        target_handle = drawTarget_(theta, Z, h, L, []);
        line_handle = drawLine_(theta, Z, h, L, []);
        
        axis([-6*L, 6*L, -11*L, 11*L]);
        xlabel('Z_v Position')
        ylabel('h Position')
        
    % at every other time step, redraw planar VTOL
    else 
        drawBody_(theta, Z, h, L, body_handle);
        drawTarget_(theta, Z, h, L, target_handle);
        drawLine_(theta, Z, h, L, line_handle);
        
        
    end
end

   
%
%=======================================================================
% drawBody_
% draw the planar VTOL body
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBody_(theta, Z, h, L, handle)
  
  pts = [...
      L/2, L/2;...
      L/2, L/16;...
      3*L/2, L/16;...
      3*L/2, L/8;...
      5*L/2, L/8;...
      5*L/2, -L/8;...
      3*L/2, -L/8;...
      3*L/2, -L/16;...
      L/2, -L/16;...
      L/2, -L/2;...
      -L/2, -L/2;...
      -L/2, -L/16;...
      -3*L/2, -L/16;...
      -3*L/2, -L/8;...
      -5*L/2, -L/8;...
      -5*L/2, L/8;...
      -3*L/2, L/8;...
      -3*L/2, L/16;...
      -L/2, L/16;...
      -L/2, L/2;...
      ]';
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:) + Z;
  Y = pts(2,:)+(L/2) + h;
  
  if isempty(handle),
    handle = fill(X,Y,'k');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%=======================================================================
% drawTarget_
% draw the Target
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawTarget_(theta, Z, h, L, handle)
 
  pts = [...
      L/8, L/4;...
      L/8, 0;...
      -L/8, 0;...
      -L/8, L/4;...
      ]';
    
  X = pts(1,:);
  Y = pts(2,:);
  X = X + (h*tan(theta)) + Z;
  
  if isempty(handle),
    handle = fill(X,Y,'r');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%=======================================================================
% drawTarget_Line
% draw the line pointing toward the Target
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawLine_(theta, Z, h, L, handle)
 
  pts = [...
      0, L/4;...
      0, L/4;...
      ]';
  
  pts(1,1) = pts(1,1) + Z;
  pts(2,1) = pts(2,1) + h;
  pts(1,2) = pts(1,2) + (h*tan(theta)) + Z;
    
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b--');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end