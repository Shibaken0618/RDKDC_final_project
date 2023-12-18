%% main script to run robot motion control

% x = pi/2;
% g_start = [cos(x) -sin(x) 0 .25;
%            -sin(x) cos(x) 0 .6;
%            0 0 -1 .22;
%            0 0 0 1];
% 
% g_end = [cos(x) -sin(x) 0 .4;
%         -sin(x) cos(x) 0 .45;
%          0 0 -1 .22;
%          0 0 0 1];
% pen_tip_offset = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
% pen_tip_offset_inv = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1]; %inverse pen tip transformation from tool tip to base_link
% theta_start = ur5InvKin(g_start * pen_tip_offset_inv);
% theta_end = ur5InvKin(g_end * pen_tip_offset_inv);

ur5 = ur5_interface();

%% Teach
% ur5.swtich_to_ros_control()
disp('Teach the start point, press any button to continue.');
% Switch to pendant control
ur5.swtich_to_pendant_control();
waitforbuttonpress;

% Record the start point
angles_start = ur5.get_current_joints();
disp('The start point joint data is:')
disp(angles_start)

% Record the end point
disp('Teach the end point, press any button to continue.');
ur5.swtich_to_pendant_control();
waitforbuttonpress;

angles_end = ur5.get_current_joints();
disp('The end point joint data is:')
disp(angles_end)

%% Draw
ur5.swtich_to_ros_control()
ur5.move_joints(ur5.home, 15);
pause(15);

% choose control type
cType = input("Choose mode: 1 -> Inv Kin Control, 2 -> RR Control, 3 -> JT Control");
while (cType ~= 1 && cType ~= 2 && cType ~= 3)
    cType = input("Invalid input:1 -> Inv Kin Control, 2 -> RR Control, 3 -> JT Control");
end
if cType == 1
    [sp_err,so_err,ep_err,eo_err] = IKControlFunc(ur5, angles_start, angles_end);
elseif cType == 2
    [sp_err,so_err,ep_err,eo_err] = RRControlFunc(ur5, angles_start, angles_end);
elseif cType == 3
    [sp_err,so_err,ep_err,eo_err] = JTControlFunc(ur5, angles_start, angles_end);
end

%display location and orientation error
disp(['Start Orientation Error: ', num2str(so_err)])
disp(['Start Position Error: ', num2str(sp_err)])
disp(['End Orientation Error: ', num2str(eo_err)])
disp(['End Position Error: ', num2str(ep_err)])

