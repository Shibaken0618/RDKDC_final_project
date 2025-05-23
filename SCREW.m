% function that finds the screw motion using xi and angle

function screw = SCREW(xi,theta)
    v = xi(1:3); w = xi(4:6);
    I3 = eye(3);
    R = I3 + (SKEW3(w) * sin(theta)) + ...
        (SKEW3(w)^2 * (1 - cos(theta)));
    p = (I3 - R) * (cross(w, v) + (w * transpose(w) * v * theta));
    screw = [R, p; zeros(1,3), 1];
end