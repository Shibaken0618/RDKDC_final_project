% function that finds the screw motion using xi and angle

function rot = SCREW(xi,theta)
    w = xi(4:6); r = xi(1:3);
    I3 = eye(3);
    R = I3 + (SKEW3(w) * sin(theta)) + ...
        (SKEW3(w)^2 * (1 - cos(theta)));
    p = -SKEW3(w) * r;
    rot = [R, p; zeros(1,3), 1];
end