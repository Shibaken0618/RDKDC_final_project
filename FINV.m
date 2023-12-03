% function for calculating inverse of 4x4 homogenous matrix

function invMat = FINV(g)
    % find new rotation and translation
    R = transpose(g(1:3, 1:3));
    p = -R * g(1:3, 4);
    % concatenate to new matrix
    invMat = [R, p; zeros(1,3), 1];
end