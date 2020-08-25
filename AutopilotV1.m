clc; clear; close all;

Vo = 10;

lambda = @(X,Y) Vo/(X^2 + Y^2);

rx = @(theta,dldx)  (  -2*cos(theta)*(1/dldx)  )^(1/3);
ry = @(theta,dldy)  (  -2*sin(theta)*(1/dldy)  )^(1/3);

L = -20:0.1:20;
len = length(L);

[X, Y] = meshgrid(L);
for i=1:length(X)
    for j=1:length(Y)
        Z(i,j) = lambda(X(i,j) ,Y(i,j) );
        if isinf(Z(i,j)) || Z(i,j) > Vo
            Z(i,j) = Vo;
        end
    end
end
h = figure; hold all;
mesh(X,Y,Z);
c = colorbar;
c.Label.String = 'Strength of Thermal';
xlabel('x-position')
ylabel('y-position')

nptx(1) = -10;
npty(1) = -10;
i = find(X(1,:) == nptx);
j = find(Y(:,j) == npty);
ao = 0;
global dldx
global dldy
for t=1:100
    
    [dldx, dldy] = findLambdas(i,j, Z);
    %angle_to_center = @(theta)  1/dldy*sin(theta) - 1/dldx*cos(theta);
            if dldx == 0
                ao = fzero( @(x) sin(-pi/2 - pi*x) , ao);
                disp('dldx is zero')
            elseif dldy == 0
                ao = fzero( @(x) sin(-pi - pi*x) , -ao);
                disp('dldy is zero')
            else
                ao = fzero(@angleToCenter, ao );
            end
    theta(t) = ao;
    r(t) = (  rx(theta(t),dldx)  );
    angle_of_tangent = theta(t) + pi/2;
    [nptx(t+1), npty(t+1), i, j] = findNextPoint(angle_of_tangent, X, Y, nptx(t), npty(t), i, j);

end
filename = 'AutopilotGif.gif';
title('Path of Glider using Autopilot ')
%zlabel('Thermal Strength')
ylabel('y-coordinate')
xlabel('x-coordinate')
hold all;
for i=1:length(nptx)
    plot(nptx(i),npty(i), 'ro-')
    drawnow
    view([0 -90]);
    pause(0.05);
          % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
end




function [result] = angleToCenter(theta)
    global dldy
    global dldx
    
    result = 1/dldy*sin(theta) - 1/dldx*cos(theta);

end

function [dldx, dldy] = findLambdas(i, j, Z)
    y2 = Z(i,j+1);
    y1 = Z(i,j-1);
    x2 = Z(i+1,j);
    x1 = Z(i-1,j);
    dldx = x2 - x1;
    dldy = y2 - y1;
end

function [nptx, npty, i, j] = findNextPoint(angle_of_tangent, X, Y, ptx, pty, i, j)

    dy = sin(angle_of_tangent);
    dx = cos(angle_of_tangent);
    
    X(:,i) = 100*ones(length(X),1);
    Y(j,:) = 100*ones(1,length(Y));

    temp = abs( X(1,:)-(ptx+dx) );
    [x, i ] = min(temp);
    
    temp = abs( Y(:,1)-(pty+dy) );
    [y, j]=min(temp);
    
    nptx = X(1,i);
    npty = Y(j,1);
    


end

