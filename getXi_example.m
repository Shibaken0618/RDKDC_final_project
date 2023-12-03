function xi = getXi(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    theta = acos((trace(R) - 1)/2);
    disp(theta)

    if theta == 0
        w = [0;0;0];
        v = p/norm(p,2);
        theta = norm(p,2);
        xi = [v;w] * theta;
    else
        w = INVHAT3((1/(2*sin(theta))) * (R- R.'));
        v = p\(eye(3)*theta + (1-cos(theta)*SKEW3(w) + (theta - sin(theta))*SKEW3(w)^2));
        v1 = p\((eye(3)-R)*SKEW3(w)+w*w.'*theta);
        disp(v)
        disp(v1)
        xi = [v.';w] * theta;
    end
end