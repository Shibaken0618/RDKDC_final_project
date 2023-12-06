%% Lab 3
% This file contains the tests for functions needed to control the robot.
% It is recommended to run each test individually to prevent overriding of
% variables.

%% (a) ur5FwdKin test

clear; clc; close all;
ur5 = ur5_interface();

% different angles for experimentation
% q = [0; -pi/4; pi/8; 0; pi/8; 0];  % pose 1 (remove comment to use, comment to not use)
% q = [pi/2; -pi/3; pi/6; pi/6; pi/2; -pi/4]; % pose 2 (remove comment to use, comment to not use) 
q = [-pi/4; -pi/5; -pi/4; -pi/3; -pi/4; pi/5]; % pose 3 (remove comment to use, comment to not use)

% create frames and move robot joints
gst = ur5FwdKin(q);
I4 = eye(4);
tool_frame = tf_frame('base_link','tool_frame',I4);
pause(1);
tool_frame.move_frame('base_link',gst);
pause(2);
ur5.move_joints(q,10);
pause(2);
gst_rviz = tool_frame.read_frame('base_link');
pause(2);

% calculate error between fwdkin and rviz
error = gst - gst_rviz;
disp(error);

%% (b) ur5BodyJacobian test

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

%% (c) manipulability test

% Initialize joint angles
theta1 = pi/2; % set your desired initial joint angles
theta2 = pi/6;
theta4 = -pi/4;
theta5 = pi;
theta6 = pi/2;

% Define the range of theta3 values
theta3_range = linspace(-pi/4, pi/4, 100);

% Initialize arrays to store manipulability values for each measure
mu_sigmamin = zeros(1, length(theta3_range));
mu_detjac = zeros(1, length(theta3_range));
mu_invcond = zeros(1, length(theta3_range));

% Loop through theta3 values
for i = 1:length(theta3_range)
    theta3 = theta3_range(i);

    % Construct joint angles vector
    joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6];

    % Compute Jacobian matrix using your UR5 Jacobian function
    J = ur5BodyJacobian(joint_angles);
    disp(J);

    % Compute manipulability for each measure
    mu_sigmamin(i) = manipulability(J, 'sigmamin');
    mu_detjac(i) = manipulability(J, 'detjac');
    mu_invcond(i) = manipulability(J, 'invcond');
end

% Plot the results
figure;

subplot(3,1,1);
plot(theta3_range, mu_sigmamin);
title('Manipulability - sigmamin');
xlabel('Theta3 (rad)');
ylabel('Manipulability');

subplot(3,1,2);
plot(theta3_range, mu_detjac);
title('Manipulability - detjac');
xlabel('Theta3 (rad)');
ylabel('Manipulability');

subplot(3,1,3);
plot(theta3_range, mu_invcond);
title('Manipulability - invcond');
xlabel('Theta3 (rad)');
ylabel('Manipulability');

%% (d) getXi test

% Define variables
str = {'1st', '2nd', '3rd'};

v = [0 0 1
     0 1 0
     1 0 0];
 
w = [1 0 0
     0 1 0
     0 0 1];

theta = zeros(3,1); 
 
% Generate rigid body motion matrix and calculate Xi.
for i = 1:3
     theta(i) = pi/(2*i);
     g(1:3, 1:3) = SKEW3(w(:, i));
     g(1:3, 4) = v(:, i);
     g(4, :) = zeros(1, 4);
     g = g.*theta(i);
     G{i} = expm(g);

     [Xi, theta_2] = getXi(G{i});
     
% Compare the value obtained from the 'getXi' function with the initial values.
     if round(Xi(1:3), 14) == round(v(:,i)*theta(i), 14)
         if round(Xi(4:6), 14) == round(w(:,i)*theta(i), 14)
            disp(['The ', char(str(i)), ' un-normalized Xi is:']);
            disp(round(Xi,2));
            disp(['The ', char(str(i)),' theta is ', num2str(theta_2)])
            disp(' ')
         end
     end

end

%% ur5RRcontrol test

clc;clear
ur5 = ur5_interface;
ur5.move_joints(ur5.home, 5);
pause(5);

% angles = [ 0.2403; -1.5708; 0; -1.5708; 0; 0];  %% The singularity position

angles = [pi/4;-pi/4;pi/12;-pi/2;pi/12;pi/12];

disp('The goal position is:');
disp(angles)

gst_star = ur5FwdKin(angles);

error = ur5RRcontrol(gst_star, 0.03, ur5_interface);
