% function for roll (3x3 rotation matrix that takes in scalar
% (radians) value

function roll = ROTX(theta)
    roll = [1, 0, 0;
            0, cos(theta), -sin(theta);
            0, sin(theta), cos(theta)];
end