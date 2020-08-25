clc; clear; close all;

radius = 300;
max_vel = 100;

[X,Y,V] = thermalprofileFull(max_vel,radius,1000);

figure;
mesh(X,Y,V)
c = colorbar;
c.Label.String = 'Strength of Thermal';

%%
% k = 3
% theta = linspace(-k/100,k/100,1000);
% r = linspace(-k/100,k/100,1000);
v = 10;
% func = @(r,theta) sin(theta)/r-cos(theta)/r+v^2/r-10;
% func = @(r,theta) sin(theta)/r-10;

t = linspace(-10,10,1000);
func = @(theta) sqrt(1000*cos(theta));

for i=1:length(t)
%     for j=1:length(r)
%         az(i,j) = func(r(i),theta(j)); 
%     end
    v(i) = func(t(i));
end
plot(t,v)
% mesh(r,theta,az)