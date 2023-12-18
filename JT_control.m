clc;clear
ur5 = ur5_interface();

%% Teach
ur5.swtich_to_ros_control();
disp('Teach the start position, then press any key to continue.');
% Switch to pendant control
ur5.swtich_to_pendant_control();
waitforbuttonpress;

% Get the start point
angles_start = ur5.get_current_joints();
disp('The start point is:')
disp(angles_start)
disp('Teach the start position, then press any key to continue.');

% Get the end point
ur5.swtich_to_pendant_control();
waitforbuttonpress;
angles_end = ur5.get_current_joints();
disp('The end point is:')
disp(angles_end)
ur5.swtich_to_ros_control();

ur5.move_joints(ur5.home, 15);
pause(15);

ur5.move_joints(angles_start, 20);
pause(20)

%% Draw
g_start = ur5FwdKin_DH(angles_start);
g_end = ur5FwdKin_DH(angles_end);

K = 0.1;
t_step = 1;
t_interval = 1;
finalerr = 0;

q_k = ur5.get_current_joints();

gst_star = g_end;

p_star = gst_star(1:3, 4);  % The goal translation
[~, theta_star] = getXi(gst_star);

gst_present = ur5FwdKin_DH(q_k);  % The present rigid body motion matrix
p_present = gst_present(1:3, 4);
[~, theta_present] = getXi(gst_present);

exp_xi_k = inv(gst_star)*gst_present;  % Error between the goal point and the start point
[xi_k, ~] = getXi(exp_xi_k);

while norm(p_present - p_star) >= 0.005 || abs(theta_present - theta_star) >= 15*pi/180
    if abs(manipulability(ur5BodyJacobian(q_k), 'detjac')) <0.00001
        finalerr = -1;  %% Abort and return -1
        break
    end

    J = ur5BodyJacobian(q_k);
    TJ = J';
    q_k1 = q_k - K*t_step*TJ*xi_k;   %% q_k1 represents q_k+1 which is the next point
    q_k = q_k1;

    ur5.move_joints(q_k, t_interval);
    pause(t_interval);
 
    gst_present = ur5FwdKin(q_k);
    p_present = gst_present(1:3, 4);
    [~, theta_present] = getXi(gst_present);

    exp_xi_k = inv(gst_star)*gst_present;
    [xi_k, ~] = getXi(exp_xi_k); 

    q_current = ur5.get_current_joints;
    disp('Current joints are:')
    disp(q_current)
end

if finalerr == -1
    warning('Matrix is close to being singular. Aborting.');
    disp(finalerr);
else
    finalerr = norm(p_present - p_star);
    disp(['The final positional error is ', num2str(finalerr*100), 'cm'])
end

ur5.move_joints(ur5.home, 10);
pause(10);
