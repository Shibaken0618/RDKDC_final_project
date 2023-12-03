function P = Trans(v, w, theta)

[rows, cols] = size(v);
if ((rows ~= 3) || (cols ~= 1))
    error('Translation matrix requires a 3x1 vector');
end

P = ((eye(3) - expm(SKEW3(w)*theta))*SKEW3(w) + w*w'*theta)*v;
% P = (theta*eye(3) + (1-cos(theta))*SKEW3(w) + (theta - sin(theta))*SKEW3(w)^2)*v;

end