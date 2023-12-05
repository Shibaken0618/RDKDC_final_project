%% lab 3 function tests

clear; clc; close all;

ur5 = ur5_interface();
% rosinit("RVIZ");
q = [0; pi/4; -pi/2; 0; pi/2; 0];
% q2 = [pi/2; -pi/3; pi/3; pi/3; pi/2; pi/2];
% q3 = [0, 0, 0, 0, 0, 0]';
% q4 = [0; pi/6; pi/3; -pi/4; pi/5; pi/6];

gst = ur5FwdKin(q);

pause(2);
tool_frame = tf_frame('base_link','tool_frame',eye(4));
pause(2);
tool_frame.move_frame('base_link',gst);
pause(2);
time_interval = 10; 
ur5.move_joints(q, time_interval);
pause(time_interval);
