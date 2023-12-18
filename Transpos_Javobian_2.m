clc;clear
ur5 = ur5_interface();

% disp('Teach the start position, then press any key to continue.');
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% % 
% % Get the start joint angles
% alpha = ur5.get_current_joints();
% disp('Teach the start position, then press any key to continue.');
% % 
% % Switch to pendant control
% ur5.swtich_to_pendant_control();
% waitforbuttonpress;
% % 
% % Get the target joint angles
% theta = ur5.get_current_joints();
% disp('Switch to ros control.');
% ur5.swtich_to_ros_control();

pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

% alpha = [   0.8727
%            -1.1083
%             1.7862
%            -2.2864
%            -1.6755
%             0];
% 
% theta = [0.8552
%         -1.2654
%          2.1118
%         -2.3387
%         -1.6755
%          0];
% 
% g_end = ur5FwdKin(theta);
% g_start = ur5FwdKin(alpha);

% g_start = [    0.7593   -0.6423    0.1042    0.3481
%                -0.6423   -0.7655   -0.0385    0.5516
%                 0.1044   -0.0377   -0.9938    0.3
%                      0         0         0    1.0000];  %% Pen-tip start point
% 
% g_end = [    0.7559   -0.6540    0.0277    0.2715
%            -0.6463   -0.7524   -0.1274    0.4463
%             0.1042    0.0784   -0.9915   0.3
%                  0         0         0    1.0000];  %% Pen-tip end point

g_start =[

    0.9255   -0.3784   -0.0180    0.2531
   -0.3774   -0.9251    0.0430    0.5958
   -0.0329   -0.0330   -0.9989    0.3667
         0         0         0    1.0000];

g_end =[

    0.9760    0.2142   -0.0393   -0.1302
    0.2153   -0.9762    0.0254    0.6341
   -0.0329   -0.0332   -0.9989    0.3668
         0         0         0    1.0000];

% theta = ur5InvKin(g_end);

start_frame = tf_frame('base_link','start',eye(4));  %% Pen-tip start point frame
pause(1);
start_frame.move_frame('base_link',g_start);

end_frame = tf_frame('base_link','end',eye(4));  %% Pen-tip end point frame
pause(1);
end_frame.move_frame('base_link',g_end);

pen_tip_frame = tf_frame('tool0','pen tip',pen_tip_offset1);  %% Pen-tip frame

%% Move and record

% ur5.move_joints(ur5.home, 20);
% pause(20);
alpha = ur5InvKin(g_start * pen_tip_offset2);  %% End-effecotr start point
q_current = ur5.get_current_joints();  %% End-effector start angle
[~, min_error_i] = min(vecnorm(alpha - q_current,1));
ur5.move_joints(alpha(:,min_error_i), 10);
pause(10);

theta = ur5InvKin(g_end * pen_tip_offset2);
q_current = ur5.get_current_joints();  %% End-effector start angle
[~, min_error_i] = min(vecnorm(theta - q_current,1));
ur5.move_joints(theta(:,min_error_i), 10);
pause(10);

K = 0.1;

t_step = 1;

finalerr = 0;

q_k = ur5.get_current_joints();

gst_star = g_end * pen_tip_offset2;  %% End-effector end point

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

    ur5.move_joints(q_k, 1);
    pause(1);
 
    gst_present = ur5FwdKin_DH(q_k);
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
