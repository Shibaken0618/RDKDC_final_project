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
ur5.move_joints(ur5.home, 20);
pause(20);

%put frames at start and end
start_frame = tf_frame('base_link','start',eye(4));
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));
pause(1);
end_frame.move_frame('base_link',g_end);

g_10cm = [1 0 0 .10; 0 1 0 0; 0 0 1 0; 0 0 0 1];

frame_10cm = tf_frame('base_link','10cm offset origin',eye(4));
pause(1);
frame_10cm.move_frame('base_link',g_start * g_10cm);


