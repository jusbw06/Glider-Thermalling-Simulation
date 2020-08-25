function [upwardvelocity] = thermalprofilePt(center_velocity,radius,x,y)
% This function creates a polar matrix plotting a velocity surface for a
% thermal based on its inner strength, outer strength, radius, and the
% order to get a proper profile

%   The code uses a cosine profile to approximate the bell-curve profile of the
%   surface. 

% Inputs are as follows: center upward velocity, thermal radius,
% xpoint, ypoint 
% Outputs are as follows: upward velocity matrix, upward velocity at the given point 

% for an single point

    r = sqrt((x^2) + (y^2)) ;
    upwardvelocity = ((center_velocity/2) * cos(((r)/radius) * pi)) + (center_velocity/2) ;

end
