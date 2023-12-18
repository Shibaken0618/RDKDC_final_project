%% main function for RR Control

function [sp_err,so_err,ep_err,eo_err] = JTControlFunc(ur5, theta_start, theta_end)
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);

    ur5.move_joints(theta_start, 15);
    pause(15)

    [so_err,sp_err] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    
    error_end = ur5JTcontrol(g_end, 0.05, ur5_interface);

    [eo_err,ep_err] = locationError(g_end, ur5FwdKin_DH(ur5.get_current_joints()));
end
