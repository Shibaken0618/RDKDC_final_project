function v = vector(R)

[rows, cols] = size(R);
if ((rows ~= 3) || (cols ~= 3))
    error('This function(vector) needs a 3X3 rotation matrix as input');
end

v = zeros(3,1);
v(1) = R(3,2);
v(2) = R(1,3);
v(3) = R(2,1);

end

