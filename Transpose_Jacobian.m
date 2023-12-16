ur5 = ur5_interface();

% disp('Teach the start position, then press any key to continue.');
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% 
% % Get the start joint angles
% alpha = ur5.get_current_joints();
% disp('Teach the start position, then press any key to continue.');
% 
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% 
% % Get the target joint angles
% theta = ur5.get_current_joints();
% disp('Switch to ros control.');

alpha = [    0.8686
   -1.4435
    1.6247
   -1.7839
   -1.4066
   -0.2459];

theta = [0.5896
-1.4436
1.6245
-1.7837
-1.4064
-0.2459];

g_end = ur5FwdKin(theta);

ur5.move_joints(ur5.home, 10);
pause(10);

ur5.move_joints(alpha, 10);
pause(10);

K = 0.1;

t_step = 1;
finalerr = 0;

q_k = ur5.get_current_joints();

gst_star = g_end;

p_star = gst_star(1:3, 4);  % The goal translation
[~, theta_star] = getXi(gst_star);

gst_present = ur5FwdKin(q_k);  % The present rigid body motion matrix
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

    ur5.move_joints(q_k, t_step);
    pause(t_step);
 
    gst_present = ur5FwdKin(q_k);
    p_present = gst_present(1:3, 4);
    [~, theta_present] = getXi(gst_present);

    exp_xi_k = inv(gst_star)*gst_present;
    [xi_k, ~] = getXi(exp_xi_k);
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
