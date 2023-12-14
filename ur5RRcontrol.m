function finalerr = ur5RRcontrol(gdesired, K, ur5_interface)
ur5 = ur5_interface;

t_step = 5;
finalerr = 0;

q_k = ur5.get_current_joints();

%q_k = [pi/12;-pi/2;pi/12;-pi/2;pi/12;pi/12];
%ur5.move_joints(q_k, 5);  %% Deviate the home position to avoid sigularity
%pause(5)

gst_star = gdesired;

p_star = gst_star(1:3, 4);  %% The goal translation
[~, theta_star] = getXi(gst_star);

gst_present = ur5FwdKin(q_k);  %% The present rigid body motion matrix
p_present = gst_present(1:3, 4);
[~, theta_present] = getXi(gst_present);

exp_xi_k = inv(gst_star)*gst_present;  %% Error between the goal point and the start point
[xi_k, ~] = getXi(exp_xi_k);

while norm(p_present - p_star) >= 0.005 || abs(theta_present - theta_star) >= 15*pi/180
    if abs(manipulability(ur5BodyJacobian(q_k), 'detjac')) <0.00001
        finalerr = -1;  %% Abort and return -1
        break
    end

    q_k1 = q_k - K*t_step*inv(ur5BodyJacobian(q_k))*xi_k;   %% q_k1 represents q_k+1 which is the next point
    q_k = q_k1;
    
    ur5.move_joints(q_k, t_step);
    pause(t_step)

    gst_present = ur5FwdKin(q_k);
    p_present = gst_present(1:3, 4);
    [~, theta_present] = getXi(gst_present);
    
    exp_xi_k = inv(gst_star)*gst_present;
    [xi_k, ~] = getXi(exp_xi_k);
    disp('Current angles are:')
    disp(ur5.get_current_joints())
end

if finalerr == -1
    warning('Matrix is close to being singular. Aborting.');
    disp(finalerr);
else
    finalerr = norm(p_present - p_star);
    disp(['The final positional error is ', num2str(finalerr*100), 'cm'])
end

end
