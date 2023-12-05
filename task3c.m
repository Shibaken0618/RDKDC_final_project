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