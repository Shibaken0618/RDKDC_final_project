%% Main function for Inverse Kinematics Control

function [sp_err,so_err,ep_err,eo_err] = IKControlFunc(ur5, theta_start, theta_end)
    %get transformation matrices
    g_start = ur5FwdKin_DH(theta_start);
    g_end = ur5FwdKin_DH(theta_end);
    %initialize pen offset matrices to use
    pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
    pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];
    %mutliply pen tip offset to tool frame
    pen_start = g_start * pen_tip_offset1;
    pen_end = g_end * pen_tip_offset1;
    % pen_end(1:3, 1:3) = pen_start(1:3, 1:3);
    
    %calculate intermediate points for open rectangle
    [pen_corner1, pen_corner2] = intermediatePointCalc(pen_start,pen_end);
    
    %ur5.move_joints(ur5.home, 10);
    %pause(10)
    
    %move to start position and record start location error
    ur5.move_joints(theta_start, 20);
    pause(20)
    q_current = ur5.get_current_joints();
    g_start_now = ur5FwdKin_DH(q_current);
    [so_err,sp_err] = locationError(g_start,g_start_now);
    
    %create num points between start and corner1 point
    num = 150;
    t_interval = 0.2;
    delta_pen1 = (pen_corner1 - pen_start) / num;
    %move through points that connect start and corner1 points
    for i = 1:num
        q_current = ur5.get_current_joints(); %get current joint angles

        if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
            finalerr = -1;  %% Abort and return -1
            break
        end
        
        pen_mid1{i} = pen_start + delta_pen1 * i; %calculate midpoint to travel to
        angles_mid1 = ur5InvKin(pen_mid1{i} * pen_tip_offset2); %angles to move to ith midpoint
        [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
        angles_mid = angles_mid1(:,min_error_i);
        ur5.move_joints(angles_mid, t_interval); %move to mid point i
        pause(t_interval)
    end 
    
    %same algorithm as above that moves from corner1 to corner2
    delta_pen2 = (pen_corner2 - pen_corner1) / num;
    for i = 1:num
        q_current = ur5.get_current_joints();
        
        if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
            finalerr = -1;  %% Abort and return -1
            break
        end
        
        pen_mid2{i} = pen_corner1 + delta_pen2 * i;
        angles_mid2 = ur5InvKin(pen_mid2{i} * pen_tip_offset2);
        [~, min_error_i] = min(vecnorm(angles_mid2 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
        angles_mid = angles_mid2(:,min_error_i);
        ur5.move_joints(angles_mid, t_interval);
        pause(t_interval)
    end
    
    %same algorithm as above but move from corner2 to end point
    delta_pen3 = (pen_end - pen_corner2) / num;
    for i = 1:num
        q_current = ur5.get_current_joints();
        
        if abs(manipulability(ur5BodyJacobian(q_current), 'detjac')) <0.00001
            finalerr = -1;  %% Abort and return -1
            break
        end
        
        pen_mid3{i} = pen_corner2 + delta_pen3 * i;
        angles_mid3 = ur5InvKin(pen_mid3{i} * pen_tip_offset2);
        [~, min_error_i] = min(vecnorm(angles_mid3 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
        angles_mid = angles_mid3(:,min_error_i);
        ur5.move_joints(angles_mid, t_interval);
        pause(t_interval)
    end
    
    %record end location error
    q_current = ur5.get_current_joints();
    g_end_now = ur5FwdKin_DH(q_current);
    [eo_err,ep_err] = locationError(g_end,g_end_now);
    
    %ur5.move_joints(ur5.home, 10);
    %pause(10);
end
