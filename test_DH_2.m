%% Initialize
clc; clear
ur5 = ur5_interface;
pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

%% Teach
% ur5.swtich_to_ros_control()
disp('Teach the start point, press any button to continue.');
% Switch to pendant control
ur5.swtich_to_pendant_control();
waitforbuttonpress;

% Record the start point
angles_start = ur5.get_current_joints();
disp('The start point joint data is:')
disp(angles_start)

% Record the end point
disp('Teach the end point, press any button to continue.');
ur5.swtich_to_pendant_control();
waitforbuttonpress;

angles_end = ur5.get_current_joints();
disp('The end point joint data is:')
disp(angles_end)

%% Draw
ur5.swtich_to_ros_control()
g_start = ur5FwdKin_DH(angles_start);
g_end = ur5FwdKin_DH(angles_end);

pen_start = g_start * pen_tip_offset1;
pen_end = g_end * pen_tip_offset1;

[pen_corner1, pen_corner2] = intermediatePointCalc(pen_start,pen_end);
g_corner1 = pen_corner1 * pen_tip_offset2;
g_corner2 = pen_corner2 * pen_tip_offset2;

ur5.move_joints(ur5.home, 10);
pause(10)

ur5.move_joints(angles_start, 20);
pause(20)
q_current = ur5.get_current_joints();
g_start_now = ur5FwdKin_DH(q_current);
[dSO3_start,dR3_start] = locationError(g_start,g_start_now);

angles_mid1 = ur5InvKin(g_corner1);
[~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));
ur5.move_joints(angles_mid1(:, min_error_i), 10);
pause(10)

q_current = ur5.get_current_joints();
angles_mid2 = ur5InvKin(g_corner2);
[~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));
ur5.move_joints(angles_mid2(:, min_error_i), 10);
pause(10)

ur5.move_joints(angles_end, 10);
pause(10)
q_current = ur5.get_current_joints();
g_end_now = ur5FwdKin_DH(q_current);
[dSO3_end,dR3_end] = locationError(g_end,g_end_now);

ur5.move_joints(ur5.home, 10);
pause(10);

disp('Rotation error of the start point is:')
disp(dSO3_start)
disp('Translation error of the start point is:')
disp(dR3_start)
disp('Rotation error of the end point is:')
disp(dSO3_end)
disp('Translation error of the end point is:')
disp(dR3_end)
