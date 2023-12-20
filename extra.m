%% function for extra credit tast image
% draws a 2D heart using mathematical functions to acheive its shape

function [x, y] = extra(num)
t = linspace (-pi,pi, num); 
x = 16*sin (t).^3; 
y = 13*cos (t) - 5*cos (2*t) - 2*cos (3*t) - cos (4*t);

end