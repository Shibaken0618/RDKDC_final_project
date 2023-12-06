function finalerr = ur5RRcontrol_sam(gdesired,K,ur5)
T_step = 5; %time step
p_threshold = .01; % get within 5cm of desired possition
theta_threshold = pi/12; % get within pi/12 radians of desired orientation
finalerr = 0;
%calculate variables needed to start the resolved rate control
q_k = ur5.get_current_joints();%joint angles
[~,theta_desired] = getXi(gdesired); %desired orientation
p_desired = gdesired(1:3,4); %desired position

%check if initial posittion is not a singularity
if abs(manipulability(ur5BodyJacobian(q_k), 'detjac')) < 0.001
    % Abort and return -1
    disp(abs(manipulability(ur5BodyJacobian(q_k), 'detjac')))
    disp('Matrix is close to being singular. Aborting.');
    finalerr = -1;
    disp(['Final error = ',num2str(finalerr)]);
    return;
end

%loop until an error or convergence with g desired
while finalerr == 0
    %calculate variables neded for resolved rate control
    gst_star_inv = inv(gdesired);
    gst = ur5FwdKin(q_k); %current g
    p_k = gst(1:3,4); %current position
    %variables needed for checking if we approach g desired
    [xi_k, ~] = getXi(gst_star_inv*gst); %xi_k needed for RR control
    [~, theta_present] = getXi(gst);
    
    % keep updating joint angles until we are close to g desired
    if (norm(p_k-p_desired)) >= p_threshold || (abs(theta_present-theta_desired)) >= theta_threshold
        %calculate new joint angles using resolved rate control
        q_k1 = q_k - K*T_step * inv(ur5BodyJacobian(q_k)) * xi_k;
        %move the robot to new calculated joint angles and pause so
        %simulaton can reach position
        ur5.move_joints(q_k1,T_step); 
        pause(T_step)
        %replace old joint angles with current angles
        q_k = ur5.get_current_joints(); 
        %Check if new position is close to a singularity
        if abs(manipulability(ur5BodyJacobian(q_k), 'detjac')) < 0.001
            % Abort and return -1
            disp('Matrix is close to being singular. Aborting.');
            finalerr = -1;
            disp(['Final error = ',num2str(finalerr)]);
            return;
        end
    else
        %output final positonal error in cm
        finalerr = norm(gdesired(1:3,4) - gst(1:3,4)) * 100; 
        return;
    end
end
end
