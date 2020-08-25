clc; clear; close all;


radius = 300;
max_vel = 2.16;
resolution=1000;

[X,Y,V] = thermalprofileFull(max_vel,radius,1000);

figure; hold all;
mesh(X,Y,V)
c = colorbar;
c.Label.String = 'Strength of Thermal';
xlabel('x-position')
ylabel('y-position')
close all;
%% Stage 1
%yo = [Vx, Vy, Vz, x, y, z];
yo = [15, 15, -10, -200, -180, 1000];
[T, R] = ode45(@Stage1,[1 15],yo);
x = R(:,4); y = R(:,5); z = R(:,6);

normal = [x(end); y(end)];
unorm = normal/norm(normal);
zv = R(end,3);
xv = 23 * unorm(2);
yv = -23 * unorm(1);
yo = [xv, yv, zv, x(end), y(end), z(end)];
[T, R] = ode45(@Stage2,[1 20],yo);
x2 = vertcat(R(1,4),R(5:end,4) ); y2 = vertcat(R(1,5),R(5:end,5)); z2 = vertcat(R(1,6),R(5:end,6));
x = vertcat(x,x2); y = vertcat(y,y2); z = vertcat(z,z2);

close all;
filename = 'ParticleSimulation.gif';
h = figure;hold all;
%axis tight manual % this ensures that getframe() returns a consistent size
title('Path of a glider Entering a Thermal')
zlabel('Altitude')
ylabel('y-motion')
xlabel('x-motion')
for i=1:length(x)
    
    plot3(x(i),y(i),z(i),'-ob')
    drawnow
    view([-90 15]);
    pause(0.35);
    
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
    
%     view([66 3.7])
end

%% Functions

function [result] = Stage2(t,yo)

    Vx = yo(1); Vy = yo(2); Vz = yo(3);
    x = yo(4); y = yo(5); z = yo(6);
    
    alpha = 20;
    radius = 300;
    max_vel = 2.16;
    
    y2 = thermalprofilePt(max_vel,radius,x,y+1);
    y1 = thermalprofilePt(max_vel,radius,x,y-1);
    x2 = thermalprofilePt(max_vel,radius,x+1,y);
    x1 = thermalprofilePt(max_vel,radius,x-1,y);
    dldx = x2 - x1;
    dldy = y2 - y1;

    normal = [dldx; dldy];
    unorm = normal/norm(normal);

    m = 1;
    g = 10;

    Ac = (Vx^2+Vy^2)/sqrt(x^2+y^2);
    Ax = Ac*unorm(1);
    Ay = Ac*unorm(2);
    Az = thermalprofilePt(max_vel,radius,x,y) * alpha - m*g;

    result = [Ax;Ay;Az;Vx;Vy;Vz];
    
end


function [result] = Stage1(t,yo)

    Vx = yo(1); Vy = yo(2); Vz = yo(3);
    x = yo(4); y = yo(5); z = yo(6);
    
    alpha = 20;
    radius = 300;
    max_vel = 2.16;
    m = 1;
    g = 10;

    Ax = 0;
    Ay = 0;
    Az = thermalprofilePt(max_vel,radius,x,y) * alpha - m*g;

    result = [Ax;Ay;Az;Vx;Vy;Vz];
    
end