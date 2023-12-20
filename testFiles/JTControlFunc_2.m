%% main function for RR Control

function [sp_err,so_err,ep_err,eo_err] = JTControlFunc_2(ur5, theta_start, theta_end)
    %calculate the transformation matrices
    num = 10;
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);

    %move to start position
    ur5.move_joints(theta_start, 15);
    pause(15)
    
    %calculate start location error
    [so_err,sp_err] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    
    %move to end position using JT control
    delta_g = (g_end - g_start) / num;
    for i = 1: num
        g_mid = g_start + delta_g * i;
        error_end = ur5JTcontrol(g_mid, 1, ur5_interface);
    end
    %record end location error
    [eo_err,ep_err] = locationError(g_end, ur5FwdKin_DH(ur5.get_current_joints()));
end
