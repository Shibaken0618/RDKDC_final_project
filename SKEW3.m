% function that accepts a 3x1 vector x = [x1, x2, x3]T
% and outputs canonical 3x3 skew-symmetric matrix

function sk = SKEW3(x)
    % take each element in x individually for skew arrangement
    x1 = x(1); x2 = x(2); x3 = x(3);
    % skew matrix
    sk = [0, -x3, x2;
          x3, 0, -x1;
          -x2, x1, 0];
end