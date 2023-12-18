%% main function for RR Control

function [sp_err,so_err,ep_err,eo_err] = RRControlFunc(ur5, theta_start, theta_end)
    %calculate the transformation matrices
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);
    
    %caluclate the open rectangle corner points
    [g_corner1, g_corner2] = intermediatePointCalc(g_start,g_end);
    
    %move to start position
    ur5.move_joints(theta_start, 15);
    pause(15)
    
    %save start location error
    [so_err,sp_err] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    
    %move to both intermediate points using RR control
    errorc1 = ur5RRcontrol(g_corner1, 0.1, ur5_interface);
    errorc2 = ur5RRcontrol(g_corner2, 0.1, ur5_interface);

    %move to the end point using R control
    error_end = ur5RRcontrol(g_end, 0.05, ur5_interface);
    %record end location error
    [eo_err,ep_err] = locationError(g_end, ur5FwdKin_DH(ur5.get_current_joints()));
end
