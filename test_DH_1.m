%% Initialize
clc; clear
pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

ur5 = ur5_interface;
ur5.move_joints(ur5.home, 10);
pause(10);

g_start = [ -1   0    0  -0.35
           0   1    0   0.2
           0   0   -1   0.5419
           0   0    0   1];

g_end = [ -1   0    0  -0.2
           0   1    0   0.3
           0   0   -1   0.5419
           0   0    0   1];

point_start = g_start * pen_tip_offset2;
point_end = g_end * pen_tip_offset2;

%% Display frames
start_frame = tf_frame('base_link','start',eye(4));  %% Start point frame
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));  %% End point frame
pause(1);
end_frame.move_frame('base_link',g_end);

pen_tip_frame = tf_frame('tool0','pen tip',pen_tip_offset1);  %% Pen-tip frame

%% Move and record
q_current = ur5.get_current_joints();
q_start = ur5InvKin(point_start);

[~, min_error_i] = min(vecnorm(q_start - q_current,1));

ur5.move_joints(q_start(:,min_error_i), 20);
pause(20)
waitforbuttonpress

% Record the first point
angles_start = ur5.get_current_joints();
% pause(10)

%% Move to the end point
q_current = ur5.get_current_joints();
q_end = ur5InvKin(point_end);

% find the closest matching kinematic configuration
[~, min_error_i] = min(vecnorm(q_end - q_current,1));

ur5.move_joints(q_end(:,min_error_i), 20);
pause(20)

%% Record the end point
angles_end = ur5.get_current_joints();

%% Calculates desired point
g_start = ur5FwdKin_DH(angles_start);
g_end = ur5FwdKin_DH(angles_end);

ur5.move_joints(ur5.home, 20);
pause(20)
ur5.move_joints(angles_start, 20);
pause(20)

num = 5;
delta_g = (g_end - g_start) / num;
delta_g_x = g_end(1, 4) - g_start(1,4);
delta_g_y = g_end(2, 4) - g_start(2,4);

q_current = ur5.get_current_joints();
g_mid1 = g_start;
g_mid1(1, 4) = g_mid1(1, 4) + 0.1 * delta_g_x / norm(delta_g_x);
angles_mid1 = ur5InvKin(g_mid1);
[min_error, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));
ur5.move_joints(angles_mid1(:, min_error_i), 10);
pause(10)

q_current = ur5.get_current_joints();
g_mid2 = g_mid1;
g_mid2(2, 4) = g_end(2, 4);
angles_mid2 = ur5InvKin(g_mid2);
[min_error, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));
ur5.move_joints(angles_mid2(:, min_error_i), 10);
pause(10)

ur5.move_joints(angles_end, 10);
pause(10)

ur5.move_joints(ur5.home, 10);
pause(10);

% for i = 1:num
%     q_current = ur5.get_current_joints();
%     g_mid1{i} = g_start + delta_g * i;
%     angles_mid1 = ur5InvKin(g_mid1{i});
%     [min_error, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
%     angles_mid = angles_mid1(:,min_error_i);
%     ur5.move_joints(angles_mid, 5);
%     pause(5)
% end