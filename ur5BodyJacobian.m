function J = ur5BodyJacobian(q)
%Joint Distances (m)
L = [89.2, 425, 392, 109.3, 94.75, 82.5] * 0.001;
L0 = L(1); L1 = L(2); L2 = L(3); L3 = L(4); L4 = L(5); L5 = L(6);

%Joint vector
theta1 = q(1); theta2 = q(2)+pi/2; theta3 = q(3);
theta4 = q(4)+pi/2; theta5 = q(5); theta6 = q(6);

% define base configuration gst0
R0 = [1, 0, 0; 0, 0, 1; 0, -1, 0];
p0 = [0; L3 + L5; L0 + L1 + L2 + L4];
gst0 = [R0, p0; zeros(1,3), 1];

%xi from mathematica
xi1 = [0;0;0;0;0;1];
xi2 = [-L0;0;0;0;1;0];
xi3 = [-L0-L1;0;0;0;1;0];
xi4 = [-L0-L1-L2;0;0;0;1;0];
xi5 = [L3;0;0;0;0;1];
xi6 = [-L0-L1-L2-L4;0;0;0;1;0];

%calculate transformations
g1 = expm(HAT4(xi1)* theta1); g2 = expm(HAT4(xi2)* theta2);
g3 = expm(HAT4(xi3)* theta3); g4 = expm(HAT4(xi4)* theta4);
g5 = expm(HAT4(xi5)* theta5); g6 = expm(HAT4(xi6)* theta6);
    
%calculate the adjusted xi values for the body Jacobian
xi1_prime = ADJOINTINV(g1*g2*g3*g4*g5*g6 *gst0)*xi1;
xi2_prime = ADJOINTINV(g2*g3*g4*g5*g6 *gst0)*xi2;
xi3_prime = ADJOINTINV(g3*g4*g5*g6 *gst0)*xi3;
xi4_prime = ADJOINTINV(g4*g5*g6 *gst0)*xi4;
xi5_prime = ADJOINTINV(g5*g6 *gst0)*xi5;
xi6_prime = ADJOINTINV(g6*gst0)*xi6;
    
%form the body Jacobian
J = [xi1_prime,xi2_prime,xi3_prime,xi4_prime,xi5_prime,xi6_prime];
end