function ExpRotation = EXPCR(a)

[rows, cols] = size(a);
if ((rows ~= 3) || (cols ~= 1))
    error('Skew-symmetric matrix requires a 3x1 vector');
end

if a == 0
    ExpRotation = eye(3);
else
    x = sqrt(a(1)^2+a(2)^2+a(3)^2);
    n = a./x;
    w = SKEW3(n);

    ExpRotation = eye(3) + sin(x)*w + (1-cos(x))*(w^2);
end

