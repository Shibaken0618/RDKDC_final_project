%% ur5FwdKin
% function that calculates fowrard kinematics for the UR5 robot

function T06 = ur5FwdKin_DH(angles)
% DH parameters
d1 = 0.089159;
d2 = 0;
d3 = 0;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;
a1 = 0;
a2 = -0.425;
a3 = -0.39225;
a4 = 0;
a5 = 0;
a6 = 0;
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;

%DH transformation matrices
T01 = DH(a1, alpha1, d1, pi + angles(1));
T12 = DH(a2, alpha2, d2, angles(2));
T23 = DH(a3, alpha3, d3, angles(3));
T34 = DH(a4, alpha4, d4, angles(4));
T45 = DH(a5, alpha5, d5, angles(5));
T56 = DH(a6, alpha6, d6, angles(6));
% T6P = DH(.12228,0,.049,0); % tool0 to pen tip trasnformation using DH params

% return T06
T06 = T01 * T12 * T23 * T34 * T45 * T56;

end

