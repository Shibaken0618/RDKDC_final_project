%% Using original FwdKin
clc; clear
errors = zeros(6,1);

ur5 = ur5_interface();
ur5.move_joints(ur5.home, 20);
pause(20);

q = (rand(6,1) * 2*pi) - pi;
g_start = ur5FwdKin_DH(q);

pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1]; %inverse pen tip transformation from tool tip to base_link

start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

pen_tip_frame = tf_frame('tool0','pen tip',pen_tip_offset1); % display pen tip

q_sol = ur5InvKin(g_start * pen_tip_offset2);% multiply inverse transform

[min_error, min_error_i] = min(vecnorm(q - q_sol,1));  %% Find the best path
errors = errors + abs(q_sol(:,min_error_i) - q);

ur5.move_joints(q_sol(:,min_error_i), 20);
pause(20)

%error between pen tip and g_start
ur5.get_current_transformation("pen tip", "base_link") - ur5.get_current_transformation("start", "base_link")