function G = RigidBodyMotion(v,w,theta)

G(1:3, 1:3) = EXPCR(theta*w);
G(1:3, 4) = Trans(v, w, theta);
G(4, 1:4) = [0, 0, 0, 1];

end

