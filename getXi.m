function [xi, theta] = getXi(G)  %% Returns normalized twist(xi) and theta

[rows, cols] = size(G);
if ((rows ~= 4) || (cols ~= 4))
    error('This function(getXi) needs a 4X4 homogeneous matrix as input');
end

xi = zeros(6,1);
R = G(1:3, 1:3);  %% Rotation matrix
p = G(1:3, 4);  %% Translation matrix

theta = acos((trace(R) - 1) / 2);

if theta < 1e-4  %% Pure translation
    w = [0;0;0];
    v = p/norm(p);
    theta = norm(p);
else
    w = 1/(2*sin(theta)) * vector(R-R');
%     P = ((eye(3) - expm(SKEW3(w)*theta))*SKEW3(w) + w*w'*theta)*v
    v = inv((eye(3) - expm(SKEW3(w)*theta))*SKEW3(w) + w*w'*theta)*p;
end

xi(1:3) = v*theta;
xi(4:6) = w*theta;

end

