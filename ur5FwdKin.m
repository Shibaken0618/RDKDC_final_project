%% ur5FwdKin
% function that calculates fowrard kinematics for the UR5 robot

function gst = ur5FwdKin(angles)
% define link lengths l0 to l5 and angles
L = [89.2, 425, 392, 109.3, 94.75, 82.5] * 0.001;
L0 = L(1); L1 = L(2); L2 = L(3); L3 = L(4); L4 = L(5); L5 = L(6);

theta1 = angles(1); theta2 = angles(2); theta3 = angles(3);
theta4 = angles(4); theta5 = angles(5); theta6 = angles(6);

% define base configuration gst0
R0 = [1, 0, 0; 0, 0, -1; 0, 1, 0];
p0 = [0; L3 + L5; L0 + L1 + L2 + L4];
gst0 = [R0, p0; zeros(1,3), 1];

% define twists
xi_1 = [0; 0; 0; 0; 0; 1];
xi_2 = [0; 0; L0; 0; 1; 0]; 
xi_3 = [0; 0; L0 + L1; 0; 1; 0];
xi_4 = [0; 0; L0 + L1 + L2; 0; 1; 0];
xi_5 = [0; L3; L0 + L1 + L2; 0; 0; 1];
xi_6 = [0; L3; L0 + L1 + L2 + L4; 0; 1; 0];

% define screw motion
g1 = SCREW(xi_1, theta1); g2 = SCREW(xi_2, theta2);
g3 = SCREW(xi_3, theta3); g4 = SCREW(xi_4, theta4);
g5 = SCREW(xi_5, theta5); g6 = SCREW(xi_6, theta6);

% return gst
gst = g1 * g2 * g3 * g4 * g5 * g6 * gst0;

end

