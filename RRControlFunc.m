%% main function for RR Control

function [sp_err,so_err,ep_err,eo_err] = RRControlFunc(ur5, theta_start, theta_end)
    %calculate the transformation matrices
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);

    %initialize pen offset matrices to use
    pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
    pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];
 
    pen_start = g_start * pen_tip_offset1;
    pen_end = g_end * pen_tip_offset1;
    
    pen_end(1:3, 1:3) = pen_start(1:3, 1:3);

    %caluclate the open rectangle corner points
    [g_corner1, g_corner2] = intermediatePointCalc(pen_start,pen_end);
    g_corner1 = g_corner1 * pen_tip_offset2;
    g_corner2 = g_corner2 * pen_tip_offset2;
    
    %move to start position
    ur5.move_joints(theta_start, 15);
    pause(15)
    
    %save start location error
    [so_err,sp_err] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    
    %move to both intermediate points using RR control
    errorc1 = ur5RRcontrol(g_corner1, 0.1, ur5_interface);
    errorc2 = ur5RRcontrol(g_corner2, 0.1, ur5_interface);

    %move to the end point using R control
    error_end = ur5RRcontrol(g_end, 0.1, ur5_interface);
    %record end location error
    [eo_err,ep_err] = locationError(g_end, ur5FwdKin_DH(ur5.get_current_joints()));
end
