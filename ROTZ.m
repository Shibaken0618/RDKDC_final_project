% function for yaw (3x3 rotation matrix) that takes in scalar 
% (radians) value

function yaw = ROTZ(theta)
    yaw = [cos(theta), -sin(theta), 0;
           sin(theta), cos(theta), 0;
           0, 0, 1];
end