g_start = [0 -1 0 .25;
           -1 0 0 .6;
           0 0 -1 .22;
           0 0 0 1];

g_end = [0 -1 0 .4;
        -1 0 0 .45;
         0 0 -1 .22;
         0 0 0 1];

%% ur5RRcontrol test

ur5 = ur5_interface;
%ur5.move_joints(ur5.home, 15);
%pause(15);

%put frames at start and end
start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));
pause(1);
end_frame.move_frame('base_link',g_end);

theta_start = ur5InvKin(g_start);

ur5.move_joints(theta_start(:,6), 5);
pause(5)

disp('The goal position is:');
disp(g_end)

error = ur5RRcontrol(g_end, 0.05, ur5_interface);
