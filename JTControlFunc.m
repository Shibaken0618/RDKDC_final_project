%% main function for RR Control

function [sp_err,so_err,ep_err,eo_err] = JTControlFunc(ur5, theta_start, theta_end)
    %calculate the transformation matrices
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);
    pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
    pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

    pen_start = g_start * pen_tip_offset1; % transformation matrix of pen-tip start point
    pen_end = g_end * pen_tip_offset1;  % transformation matrix of pen-tip end point

    pen_end(1:3, 1:3) = pen_start(1:3, 1:3);

    g_end = pen_end * pen_tip_offset2;

    %move to start position
    pause(2)
    ur5.move_joints(theta_start, 15);
    pause(15)
    
    %calculate start location error
    [so_err,sp_err] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    
    %move to end position using JT control
    error_end = ur5JTcontrol(g_end, 0.5, ur5_interface);
    
    %record end location error
    [eo_err,ep_err] = locationError(g_end, ur5FwdKin_DH(ur5.get_current_joints()));
end
