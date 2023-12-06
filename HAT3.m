%creates skew symmetric matrix of a 3x1 vector
function skew3 = HAT3(v)
    skew3 = [0,-v(3),v(2);
             v(3),0,-v(1);
            -v(2),v(1),0];
end
