ur5 = ur5_interface();

% disp('Teach the start position, then press any key to continue.');
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% 
% % Get the start joint angles
% alpha = ur5.get_current_joints();
% disp('Teach the start position, then press any key to continue.');
% 
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% 
% % Get the target joint angles
% theta = ur5.get_current_joints();
% disp('Switch to ros control.');

alpha = [    0.8686
   -1.4435
    1.6247
   -1.7839
   -1.4066
   -0.2459];

theta = [    0.5896
   -1.4436
    1.6245
   -1.7837
   -1.4064
   -0.2459];

% Define control gain
lambda = 0.1;

% Define the number of iterations and step size
numIterations = 100;
stepSize = 0.01;

% Switch to ros control
% ur5.swtich_to_ros_control();
ur5.move_joints(ur5.home, 10);
pause(10);
ur5.move_joints(alpha,10);
pause(10);

% Control loop
for i = 1:numIterations
    % Calculate current end-effector position
    currentAngles = ur5.get_current_joints();
    % currentPos = ur5FwdKin(currentAngles);

    % Calculate end-effector position error
    error = theta - currentAngles;

    % Calculate Jacobian matrix
    J = ur5BodyJacobian(currentAngles);

    % Use the transpose Jacobian control law to compute joint velocities
    deltaQ = lambda * J' * error;

    % Update joint angles
    currentJointAngles = currentAngles + deltaQ;

    % Send joint angles to the robot
    ur5.move_joints(currentJointAngles,lambda);
    pause(lambda);
end

ur5.move_joints(ur5.home, 10);
pause(10);
