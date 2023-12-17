x = pi/2;
g_start = [cos(x) -sin(x) 0 .25;
           -sin(x) cos(x) 0 .6;
           0 0 -1 .22;
           0 0 0 1];

g_end = [cos(x) -sin(x) 0 .4;
        -sin(x) cos(x) 0 .45;
         0 0 -1 .22;
         0 0 0 1];

%% ur5RRcontrol test

ur5 = ur5_interface();
ur5.move_joints(ur5.home, 15);
pause(15);

%put frames at start and end
start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));
pause(1);
end_frame.move_frame('base_link',g_end);

[g_corner1, g_corner2] = intermediatePointCalc(g_start,g_end);

corner1_frame = tf_frame('base_link','corner1',eye(4));
pause(1);
corner1_frame.move_frame('base_link',g_corner1);

corner2_frame = tf_frame('base_link','corner2',eye(4));
pause(1);
corner2_frame.move_frame('base_link',g_corner2);

pen_tip_offset = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
pen_tip_offset1 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1]; %inverse pen tip transformation from tool tip to base_link

pen_tip = tf_frame('tool0','pen tip',pen_tip_offset);
pause(1);

theta_start = ur5InvKin(g_start*pen_tip_offset1);

ur5.move_joints(theta_start(:,6), 15);
pause(15)
[start_orientation_error,start_position_error] = locationError(g_start, ur5FwdKin(ur5.get_current_joints()));
disp(['Start Orientation Error: ', num2str(start_orientation_error)])
disp(['Start Position Error: ', num2str(start_position_error)])


error1 = ur5RRcontrol(g_corner1, 0.05, ur5_interface);

error2 = ur5RRcontrol(g_corner2, 0.05, ur5_interface);

disp('The goal position is:');
disp(g_end)

error = ur5RRcontrol(g_end, 0.05, ur5_interface);
[end_orientation_error,end_position_error] = locationError(g_end, ur5FwdKin(ur5.get_current_joints()));
disp(['End Orientation Error: ', num2str(end_orientation_error)])
disp(['End Position Error: ', num2str(end_position_error)])
