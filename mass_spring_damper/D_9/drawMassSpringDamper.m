function drawMassSpringDamper(u)

    % process inputs to function
    %x   = u(1);
    x     = u(1);
    t       = u(2);
    
    % drawing parameters
    L = 1;
    %w = 1;
    
    % define persistent variables 
    %persistent wall_handle
    persistent mass_handle
    persistent spring_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        track_width=3;
        plot([-1*L,4*L],[0,0],'k--'); % plot track
        hold on
        wall = [...
            .25,0;...
            .25,1;...
            -.25,1;...
            -.25,0;...
            ];
        X1 = wall(:,1);
        Y1 = wall(:,2);
        fill(X1,Y1,'r')
        
            
        %wall_handle   = drawWall(x, w, []);
        mass_handle  = drawMass(x, L, []);
        spring_handle = drawSpring(x, L, []);
        axis([-1*L, 4*L, -2*L, 3*L]);
        xlabel('Z Position')
    
        
    % at every other time step, redraw mass
    else 
        %drawWall(x, w, base_handle);
        drawMass(x, L, mass_handle);
        drawSpring(x, L, spring_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawMass(x, L, handle)

  % define points on Mass (without rotation)
  pts = [...
      1.25*L, 0;...
      2.25*L, 0;...
      2.25*L, L;...
      1.25*L, L;...
      ]';
  % define translation
  %T = [x, 0];
  % rotate points
  pts(1,:) = pts(1,:) + x;
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle),
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=======================================================================
% drawSpring
% draw the spring and damper (represented as a simple line)
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function handle = drawSpring(x, L, handle)

  % define points on Spring (without rotation)
  pts = [...
      .25, L/2;...
      1.25, L/2;...
      ]';
  % define translation
  %T = [x, 0];
  % rotate points
  pts(1,2) = pts(1,2) + x;
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle),
    handle = fill(X,Y,'k','LineWidth',4);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
