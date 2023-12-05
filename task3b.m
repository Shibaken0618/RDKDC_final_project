%Body Jacobian test
epsilon = .01;
q = [pi/2;pi/6;-pi;-pi/4;pi;pi/2];
%offset vectors
e1 = [1;0;0;0;0;0];
e2 = [0;1;0;0;0;0];
e3 = [0;0;1;0;0;0];
e4 = [0;0;0;1;0;0];
e5 = [0;0;0;0;1;0];
e6 = [0;0;0;0;0;1];
%Gradient with respect to theta_i calculations
grad_g1 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e1)-ur5FwdKin(q-epsilon*e1));
grad_g2 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e2)-ur5FwdKin(q-epsilon*e2));
grad_g3 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e3)-ur5FwdKin(q-epsilon*e3));
grad_g4 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e4)-ur5FwdKin(q-epsilon*e4));
grad_g5 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e5)-ur5FwdKin(q-epsilon*e5));
grad_g6 = (1/(2*epsilon))*(ur5FwdKin(q+epsilon*e6)-ur5FwdKin(q-epsilon*e6));
%extract the xi terms
gst = ur5FwdKin(q);
prod1 = (gst)\grad_g1;
xi1 = [prod1(1:3,4);vector(prod1(1:3,1:3))];
prod2 = (gst)\grad_g2;
xi2 = [prod2(1:3,4);vector(prod2(1:3,1:3))];
prod3 = (gst)\grad_g3;
xi3 = [prod3(1:3,4);vector(prod3(1:3,1:3))];
prod4 = (gst)\grad_g4;
xi4 = [prod4(1:3,4);vector(prod4(1:3,1:3))];
prod5 = (gst)\grad_g5;
xi5 = [prod5(1:3,4);vector(prod5(1:3,1:3))];
prod6 = (gst)\grad_g6;
xi6 = [prod6(1:3,4);vector(prod6(1:3,1:3))];
%Form the approximate jacobian and calculated jacobian
Japprox = [xi1,xi2,xi3,xi4,xi5,xi6]
J = ur5BodyJacobian(q)
%Approximation Error
JacobianError = norm(Japprox-J)