clc; clear
%These coordinates are in reference to the DH parameter base not base_link
x = pi/2;
g_start = [cos(x) -sin(x) 0 .25;
           -sin(x) cos(x) 0 .6;
           0 0 -1 .22;
           0 0 0 1];

g_end = [cos(x) -sin(x) 0 .4;
        -sin(x) cos(x) 0 .45;
         0 0 -1 .22;
         0 0 0 1];
%initialize important values and set to home
ur5 = ur5_interface();
pause(1)
ur5.move_joints(ur5.home, 20);
pause(20);

errors = zeros(6,1);
pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

%change base_link frames to DH parameter frames
q_start = ur5InvKin(g_start * pen_tip_offset2);%not needed in test
g_start = ur5FwdKin_DH(q_start);
q_sol_start = ur5InvKin(g_start * pen_tip_offset2);% multiply inverse transform

%change base_link frames to DH parameter frames
q_end = ur5InvKin(g_end * pen_tip_offset2);
g_end = ur5FwdKin_DH(q_end);
q_sol_end = ur5InvKin(g_end * pen_tip_offset2);

% Find the best path
[min_error, min_error_i] = min(vecnorm(q_start - q_sol_start,1));  
errors = errors + abs(q_sol_start(:,min_error_i) - q_start);

%plot frames
start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));
pause(1);
end_frame.move_frame('base_link',g_end);
% display pen tip
pen_tip_frame = tf_frame('tool0','pen tip',pen_tip_offset1); 

%Move to the positions following best path
ur5.move_joints(q_sol_start(:,min_error_i), 20);
pause(20)

ur5.move_joints(q_sol_end(:,min_error_i), 10);
pause(10)
