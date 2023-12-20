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

% x = pi/2;
% g_start = [cos(x) -sin(x) 0 -0.25;
%            -sin(x) cos(x) 0 0.6;
%            0 0 -1 0;
%            0 0 0 1];

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
[~, min_error_i1] = min(vecnorm(q_start - q_current,1));
% ur5.move_joints(q_start(:,min_error_i1), 20);
% pause(20)

q_end = ur5InvKin(point_end);
[~, min_error_i2] = min(vecnorm(q_end - q_current,1));

[sp_err,so_err,ep_err,eo_err] = JTControlFunc(ur5, q_start(:,min_error_i1), q_end(:,min_error_i2));
