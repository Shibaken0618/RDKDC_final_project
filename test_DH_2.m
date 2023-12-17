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
% pen_end(1:3, 1:3) = pen_start(1:3, 1:3);

[pen_corner1, pen_corner2] = intermediatePointCalc(pen_start,pen_end);
% g_corner1 = pen_corner1 * pen_tip_offset2;
% g_corner2 = pen_corner2 * pen_tip_offset2;
pause(1)
ur5.move_joints(ur5.home, 20);
pause(20)

ur5.move_joints(angles_start, 20);
pause(20)
q_current = ur5.get_current_joints();
g_start_now = ur5FwdKin_DH(q_current);
[dSO3_start,dR3_start] = locationError(g_start,g_start_now);

% angles_mid1 = ur5InvKin(g_corner1);
% [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));
% ur5.move_joints(angles_mid1(:, min_error_i), 10);
% pause(10)

num = 15;
t_step = 0.5;
delta_pen1 = (pen_corner1 - pen_start) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    pen_mid1{i} = pen_start + delta_pen1 * i;
    angles_mid1 = ur5InvKin(pen_mid1{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid1(:,min_error_i);
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
end

% q_current = ur5.get_current_joints();
% angles_mid2 = ur5InvKin(g_corner2);
% [~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));
% ur5.move_joints(angles_mid2(:, min_error_i), 10);
% pause(10)

delta_pen2 = (pen_corner2 - pen_corner1) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    pen_mid2{i} = pen_corner1 + delta_pen2 * i;
    angles_mid2 = ur5InvKin(pen_mid2{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid2(:,min_error_i);
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
end

delta_pen3 = (pen_end - pen_corner2) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    pen_mid3{i} = pen_corner2 + delta_pen3 * i;
    angles_mid3 = ur5InvKin(pen_mid3{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid3 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid3(:,min_error_i);
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
end
% ur5.move_joints(angles_end, 10);
% pause(10)

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
