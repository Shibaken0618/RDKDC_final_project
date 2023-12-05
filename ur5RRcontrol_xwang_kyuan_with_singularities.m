%% Initiate
clc;clear

%% Define vatiables
K = 10;
t_step = 1;
q_k = zeros(6,1);  %% Initial angle of joints(probably wrong)
% May need to change initial angle because theta3=0 and theta5=0 are both
% singularities

if abs(manipulability(ur5BodyJacobian(q_k), 'detjac')) < 0.01
    % Abort and return -1
    disp('Matrix is close to being singular. Aborting.');
    finalerr = -1;
    return;
end

% angles = rand(6,1)*pi/6
angles = [0.3554
          0.3968
          0.3891
          0.2054
          0.3432
          0.0896];

if manipulability(ur5BodyJacobian(angles), 'detjac') <0.01
    % Abort and return -1
    disp('Matrix is close to being singular. Aborting.');
    finalerr = -1;
    return;
end

gst_star = ur5FwdKin(angles);  %% Generate the goal point randomly

p_star = gst_star(1:3, 4);  %% The goal translation
R_star = gst_star(1:3, 1:3);  %% The goal rotation
[xi_star, theta_star] = getXi(gst_star);
xi_star = xi_star * theta_star;  %% The un-normalized goal xi

gst_present = ur5FwdKin(q_k);  %% The present rigid body motion matrix which is ought to be the home position
p_present = gst_present(1:3, 4);
R_present = gst_present(1:3, 1:3);
[xi_present, theta_present] = getXi(gst_present);
xi_present = xi_present * theta_present;

exp_xi_k = inv(gst_star)*gst_present;  %% Error between the goal point and the home position of the end effector
[xi_k, theta_k] = getXi(exp_xi_k);
xi_k = xi_k * theta_k;  %% The initial un-normalized xi_k

while norm(p_present - p_star) >= 0.005 || abs(theta_present - theta_star) >= 15*pi/180
    q_k1 = q_k - K*t_step*inv(ur5BodyJacobian(q_k))*xi_k;   %% q_k1 represents q_k+1 which is the next point
    q_k = q_k1;
%     pause(0.5)
    gst_present = ur5FwdKin(q_k);
    p_present = gst_present(1:3, 4);
    R_present = gst_present(1:3, 1:3);
    [xi_presnet, theta_present] = getXi(gst_present);
    xi_present = xi_present * theta_present;
    exp_xi_k = inv(gst_star)*gst_present;
    [xi_k, theta_k] = getXi(exp_xi_k);
    xi_k = xi_k * theta_k;
    if manipulability(ur5BodyJacobian(q_k), 'detjac') <0.01
        % Abort and return -1
        disp('Matrix is close to being singular. Aborting.');
        finalerr = -1;
        return;
    end
end

disp(xi_k)
