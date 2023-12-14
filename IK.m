ur5 = ur5_interface();

disp('Teach the start position, then press any key to continue.');
% Switch to pendant control
ur5.switch_to_pendant_control();
waitforbuttonpress;

% Get the start joint angles
alpha = ur5.get_current_joints();
disp('Teach the start position, then press any key to continue.');

% Switch to pendant control
ur5.switch_to_pendant_control();
waitforbuttonpress;

% Get the target joint angles
theta = ur5.get_current_joints();
disp('Switch to ros control.');

%Target workspace
g_start = ur5FwdKin_DH(alpha - ur5.home);
g_target = ur5FwdKin_DH(theta - ur5.home);

% Stop
g_target1 = g_target;
g_target1(3,4) = g_target(1,4) + 0.01;
g_target2 = g_target;
g_target2(3,4) = g_target(2,4) + 0.005;

%Get joint configuration using IK
theta1 = ur5InvKin(g_target1);
theta2 = ur5InvKin(g_target2);

% Switch to ros control
ur5.swtich_to_ros_control();
ur5.move_joints(ur5.home, 10);
pause(10);
ur5.move_joints(alpha,10);
pause(10);
ur5.move_joints(theta1,5);
pause(5);
ur5.move_joints(theta2,5);
pause(5);
ur5.move_joints(theta,5);
pause(5);
ur5.move_joints(ur5.home, 10);
pause(10);
