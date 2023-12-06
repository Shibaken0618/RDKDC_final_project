%% Initiate
clc;clear
ur5 = ur5_interface;
ur5.move_joints(ur5.home, 5);
pause(5);

% angles = [ 0.2403; -1.5708; 0; -1.5708; 0; 0];  %% The singularity position

angles = [pi/4;-pi/4;pi/12;-pi/2;pi/12;pi/12];

disp('The goal position is:');
disp(angles)

gst_star = ur5FwdKin(angles);

error = ur5RRcontrol(gst_star, 0.05, ur5_interface);
