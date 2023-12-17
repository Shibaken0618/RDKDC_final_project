%% main function for RR Control

function RRControlFunc(ur5, theta_start, theta_end)
    ur5.swtich_to_ros_control()
    num = 50;
    pen_tip_offset = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
    pen_tip_offset1 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1]; %inverse pen tip transformation from tool tip to base_link
    g_start = ur5FwdKin(theta_start);
    g_end = ur5FwdKin(theta_end);

    
    %ur5 = ur5_interface();
    ur5.move_joints(ur5.home, 15);
    pause(15);
    [g_corner1, g_corner2] = intermediatePointCalc(g_start,g_end);

    %theta_start = ur5InvKin(g_start*pen_tip_offset1);


    ur5.move_joints(theta_start(:,6), 15);
    pause(15)
    [start_orientation_error,start_position_error] = locationError(g_start, ur5FwdKin_DH(ur5.get_current_joints()));
    disp(['Start Orientation Error: ', num2str(start_orientation_error)])
    disp(['Start Position Error: ', num2str(start_position_error)])
    
    error1 = ur5RRcontrol(g_corner1, 0.05, ur5_interface);
    
    error2 = ur5RRcontrol(g_corner2, 0.05, ur5_interface);
    
    disp('The goal position is:');
    disp(g_end)
    
    error = ur5RRcontrol(g_end, 0.05, ur5_interface);
    [end_orientation_error,end_position_error] = locationError(g_end, ur5FwdKin(ur5.get_current_joints()));
    disp(['End Orientation Error: ', num2str(end_orientation_error)])
    disp(['End Position Error: ', num2str(end_position_error)])
end
