clc; clear; close all;

Vo = 45;

lambda = @(X,Y) Vo/(X^2 + Y^2);

rx = @(theta,dldx)  (  -2*cos(theta)*(1/dldx)  )^(1/3);
ry = @(theta,dldy)  (  -2*sin(theta)*(1/dldy)  )^(1/3);

radius = 100;
max_vel = 2.16;
resolution=1000;

[X,Y,V] = thermalprofileFull(max_vel,radius,1000);

h = figure; hold all;
mesh(X,Y,V)
c = colorbar;
c.Label.String = 'Strength of Thermal';
xlabel('x-position')
ylabel('y-position')

num = 500;
nptx(1) = -50;
npty(1) = -50;
[val, i ] = min(abs(X(1,:) - nptx));
[val, j ] = min(abs(Y(:,1) - npty));
nptz(1) = V(i,j);
ao = 0;
global dldx
global dldy
for t=1:num
    
    [dldx, dldy] = findLambdas(i,j, V);
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
    [val, i ] = min(abs(X(1,:) - nptx(t+1)));
    [val, j ] = min(abs(Y(:,1) - npty(t+1)));
    nptz(t+1) = V(i,j);

end

filename = '3DAutopilotGif.gif';
title('Particle using Glider Autopilot 3D')
%zlabel('Thermal Strength')
ylabel('y-coordinate')
xlabel('x-coordinate')
hold all;
for i=1:length(nptz)
    plot3(nptx(i),npty(i), nptz(i), 'ro-')
    drawnow
    view([-29 56])
    %pause(0.01)
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

pos(1) = 0;
pos = zeros(num+1);
for t=1:num
    pos(t+1) = pos(t) + nptz(t)*Vo;
end

h = figure; hold all;
filename = '3DAutopilotGifZ.gif';
title('Particle using Glider Autopilot 3D')
zlabel('Alititude')
ylabel('y-coordinate')
xlabel('x-coordinate')

for i=1:length(nptz)
    plot3(nptx(i),npty(i), pos(i), 'ro-')
    drawnow
    view([-29 56])
    pause(0.01)
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

%% Functions
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

