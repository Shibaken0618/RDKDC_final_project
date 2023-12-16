%% Initialize
clc; clear
ur5 = ur5_interface;

pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

%% Teach
disp('Teach the start point, press any button to continue.');
% Switch to pendant control
ur5.switch_to_pendant_control();
waitforbuttonpress;

% Record the start point
angles_start = ur5.get_current_joints();
disp('The start point joint data is:')
disp(angles_start)

% Record the end point
disp('Teach the end point, press any button to continue.');
ur5.switch_to_pendant_control();
waitforbuttonpress;

angles_end = ur5.get_current_joints();
disp('The end point joint data is:')
disp(angles_end)

%% Draw
ur5.swtich_to_ros_control()
g_start = ur5FwdKin_DH(angles_start);
g_end = ur5FwdKin_DH(angles_end);

ur5.move_joints(ur5.home, 10);
pause(10)
ur5.move_joints(angles_start, 20);
pause(20)

delta_g_x = g_end(1, 4) - g_start(1,4);
delta_g_y = g_end(2, 4) - g_start(2,4);

q_current = ur5.get_current_joints();
g_mid1 = g_start;
g_mid1(1, 4) = g_mid1(1, 4) + 0.1 * delta_g_x / norm(delta_g_x);
angles_mid1 = ur5InvKin(g_mid1);
[~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));
ur5.move_joints(angles_mid1(:, min_error_i), 10);
pause(10)

q_current = ur5.get_current_joints();
g_mid2 = g_mid1;
g_mid2(2, 4) = g_end(2, 4);
angles_mid2 = ur5InvKin(g_mid2);
[~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));
ur5.move_joints(angles_mid2(:, min_error_i), 10);
pause(10)

ur5.move_joints(angles_end, 10);
pause(10)

ur5.move_joints(ur5.home, 10);
pause(10);
