clc; clear
errors = zeros(6,1);

ur5 = ur5_interface;
ur5.move_joints(ur5.home, 15);
pause(15);

q = (rand(6,1) * 2*pi) - pi;
g_start = ur5FwdKin(q);

start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

q_sol = ur5InvKin(g_start);

[min_error, min_error_i] = min(vecnorm(q - q_sol,1));  %% Find the best path
errors = errors + abs(q_sol(:,min_error_i) - q);

ur5.move_joints(q_sol(:,min_error_i), 20);
pause(20)