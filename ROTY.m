% function for pitch (3x3 rotation matrix) that takes in scalar
% (radians) value

function pitch = ROTY(theta)
    pitch = [cos(theta), 0, sin(theta);
            0, 1, 0;
            -sin(theta), 0, cos(theta)];
end