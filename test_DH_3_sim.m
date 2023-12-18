%% Initialize
clc; clear
pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

ur5 = ur5_interface;
% ur5.move_joints(ur5.home, 10);
% pause(10);

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

pen_tip_frame = tf_frame('tool0','pen tip',pen_tip_offset1);  %% Pen-tip frame

%% Move and record
q_current = ur5.get_current_joints();
q_start = ur5InvKin(point_start);

[~, min_error_i] = min(vecnorm(q_start - q_current,1));

ur5.move_joints(q_start(:,min_error_i), 20);
pause(20)

% Record the first point
angles_start = ur5.get_current_joints();
% pause(10)

%% Calculates desired point
g_start = ur5FwdKin_DH(angles_start);
pen_start = g_start * pen_tip_offset1;
% ur5.move_joints(angles_start, 20);
% pause(20)

t_step = 1;
[x, y] = extra(150);
x = x.*0.01;
y = (y - y(1)).*0.01;
points = size(x, 2);
count = 1;

for i = 1:points
    pen_extra{i} = pen_start; 
    pen_extra{i}(1, 4) = pen_start(1, 4) + x(i);
    pen_extra{i}(2, 4) = pen_start(2, 4) + y(i);

    q_current = ur5.get_current_joints();
    angles_mid1 = ur5InvKin(pen_extra{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid1(:,min_error_i);
    Frame_G = tf_frame('base_link', ['Frame_G_', num2str(count)], pen_extra{i});
    count = count+1;
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
end
