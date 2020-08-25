clc; clear; close all;

Vo = 1;

lambda = @(X,Y) Vo/(X^2 + Y^2);

[X,Y] = meshgrid(-4:1:4);
for i=1:length(X)
    for j=1:length(Y)
        V(i,j) = lambda(X(i,j),Y(i,j));
        if isinf(V(i,j))
            V(i,j) = Vo;
        end
    end    
end

figure;
mesh(X,Y,V)
title('Graph of $$1/r^2$$', 'interpreter','latex')
xlabel('x-position')
ylabel('y-position')