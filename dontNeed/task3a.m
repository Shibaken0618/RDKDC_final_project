%% lab 3 function tests

clear; clc; close all;
ur5 = ur5_interface();

% ur5FrwKin function test

% different angles for experimentation
% q = [0; -pi/4; pi/8; 0; pi/8; 0];  % pose 1 (remove comment to use, comment to not use)
% q = [pi/2; -pi/3; pi/6; pi/6; pi/2; -pi/4]; % pose 2 (remove comment to use, comment to not use) 
q = [-pi/4; -pi/5; -pi/4; -pi/3; -pi/4; pi/5]; % pose 3 (remove comment to use, comment to not use)

% create frames and move robot joints
gst = ur5FwdKin(q);
I4 = eye(4);
tool_frame = tf_frame('base_link','tool_frame',I4);
pause(1);
tool_frame.move_frame('base_link',gst);
pause(2);
ur5.move_joints(q,10);
pause(2);
gst_rviz = tool_frame.read_frame('base_link');
pause(2);

% calculate error between fwdkin and rviz
error = gst - gst_rviz;
disp(error);
