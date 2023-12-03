function adinv = ADJOINTINV(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    adinv = [[R.',-R.'*HAT3(p)];[zeros(3,3),R.']];
end