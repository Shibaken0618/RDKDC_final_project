function finalerr = ur5RRcontrol_sam(gdesired,K,ur5)
T_step = 5;
v_threshold = .050;
w_threshold = pi/12;
finalerr = 0;
q_k = ur5.get_current_joints();
[xi_desired,theta_desired] = getXi(gdesired);
xi_desired = xi_desired * theta_desired;
v_final = xi_desired(1:3);
w_final = xi_desired(4:6);
p_desired = gdesired(1:3,4);

while finalerr == 0
    gst_star_inv = inv(gdesired);
    gst = ur5FwdKin(q_k);
    p_k = gst(1:3,4);
    [xi_k, theta_k] = getXi(gst_star_inv*gst);
    xi_k = xi_k * theta_k;
    %get v and w to calculate norms and see if approaching g_desired
    v_k = xi_k(1:3);
    w_k = xi_k(4:6);
    if (norm(p_k-p_desired)) >= v_threshold || (norm(theta_k-theta_desired,1)) >= w_threshold
        (norm(p_k-p_desired))
        (norm(theta_k-theta_desired,1))
        q_k1 = q_k - K*T_step * inv(ur5BodyJacobian(q_k)) * xi_k;
        ur5.move_joints(q_k1,T_step)
        pause(T_step)
        q_k = ur5.get_current_joints()
    else
        finalerr = gdesired(1:3,4) - gst(1:3,4);
    end
end
end
