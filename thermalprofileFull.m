function [X,Y,upwardvelocity] = thermalprofileFull(center_velocity,radius, res)
% This function creates a polar matrix plotting a velocity surface for a
% thermal based on its inner strength, outer strength, radius, and the
% order to get a proper profile

%   The code uses a cosine profile to approximate the bell-curve profile of the
%   surface. 

% Inputs are as follows: center upward velocity, thermal radius,
% xpoint, ypoint 
% Outputs are as follows: upward velocity matrix

L = linspace(-radius,radius,res);
[X, Y] = meshgrid(L);
upwardvelocity = zeros(res);

    for i = 1:length(X)
        for j = 1:length(Y)

           r = sqrt((X(i,j)^2) + (Y(i,j)^2)) ;       
           upwardvelocity(i,j) =  ((center_velocity) * cos( ((r)/(2*radius)) * pi)) ;

        end    
    end

end
