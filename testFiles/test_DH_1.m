%% Initialize
clc; clear
finalerr = 0;
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
% waitforbuttonpress

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

pen_start = g_start * pen_tip_offset1;
pen_end = g_end * pen_tip_offset1;

[pen_corner1, pen_corner2] = intermediatePointCalc(pen_start,pen_end);

% ur5.move_joints(ur5.home, 10);
% pause(10)

ur5.move_joints(angles_start, 20);
pause(20)
q_current = ur5.get_current_joints();
g_start_now = ur5FwdKin_DH(q_current);
[dSO3_start,dR3_start] = locationError(g_start,g_start_now);

% angles_mid1 = ur5InvKin(g_corner1);
% [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));
% ur5.move_joints(angles_mid1(:, min_error_i), 10);
% pause(10)

num = 150;
t_step = 0.2;
count = 1;
delta_pen1 = (pen_corner1 - pen_start) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    g_current = ur5FwdKin_DH(q_current);

    if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
        finalerr = -1;  %% Abort and return -1
        break
    end

    pen_mid1{i} = pen_start + delta_pen1 * i;
    angles_mid1 = ur5InvKin(pen_mid1{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid1(:,min_error_i);
    Frame_G = tf_frame('base_link', ['Frame_G_', num2str(count)], pen_mid1{i});
    count = count+1;
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
    % if abs(angles_mid(3,4) - g_current(3, 4)) >= 0.01
    %     warning('Exceed Z limit');
    % else
    %     ur5.move_joints(angles_mid, t_step);
    %     pause(t_step)
    % end
end

% q_current = ur5.get_current_joints();
% angles_mid2 = ur5InvKin(g_corner2);
% [~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));
% ur5.move_joints(angles_mid2(:, min_error_i), 10);
% pause(10)

delta_pen2 = (pen_corner2 - pen_corner1) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    g_current = ur5FwdKin_DH(q_current);
    if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
        finalerr = -1;  %% Abort and return -1
        break
    end

    pen_mid2{i} = pen_corner1 + delta_pen2 * i;
    angles_mid2 = ur5InvKin(pen_mid2{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid2(:,min_error_i);
    Frame_G = tf_frame('base_link', ['Frame_G_', num2str(count)], pen_mid2{i});
    count = count+1;
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
    % if abs(angles_mid(3,4) - g_current(3, 4)) >= 0.01
    %     warning('Exceed Z limit');
    % else
    %     ur5.move_joints(angles_mid, t_step);
    %     pause(t_step)
    % end
end

delta_pen3 = (pen_end - pen_corner2) / num;
for i = 1:num
    q_current = ur5.get_current_joints();
    g_current = ur5FwdKin_DH(q_current);
    if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
        finalerr = -1;  %% Abort and return -1
        break
    end

    pen_mid3{i} = pen_corner2 + delta_pen3 * i;
    angles_mid3 = ur5InvKin(pen_mid3{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid3 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid3(:,min_error_i);
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
    Frame_G = tf_frame('base_link', ['Frame_G_', num2str(count)], pen_mid3{i});
    count = count+1;
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
    % if abs(angles_mid(3,4) - g_current(3, 4)) >= 0.01
    %     warning('Exceed Z limit');
    % else
    %     ur5.move_joints(angles_mid, t_step);
    %     pause(t_step)
    % end
end

% ur5.move_joints(angles_end, 10);
% pause(10)

q_current = ur5.get_current_joints();
g_end_now = ur5FwdKin_DH(q_current);
[dSO3_end,dR3_end] = locationError(g_end,g_end_now);

ur5.move_joints(ur5.home, 10);
pause(10);

if finalerr == -1
    warning('Matrix is close to being singular. Aborting.');
    disp(finalerr);
else
    disp('Rotation error of the start point is:')
    disp(dSO3_start)
    disp('Translation error of the start point is:')
    disp(dR3_start)
    disp('Rotation error of the end point is:')
    disp(dSO3_end)
    disp('Translation error of the end point is:')
    disp(dR3_end)
end
