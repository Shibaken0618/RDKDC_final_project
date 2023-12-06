%creates 4x4 hat matrix for a given 6x1 xi vector
function hat4 = HAT4(xi)
    w = xi(4:6,1);
    v = xi(1:3,1);
    w_hat = HAT3(w);
    hat4 = [[w_hat,v];[0,0,0,0]];
end
