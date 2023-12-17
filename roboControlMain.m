%% main script to run robot motion control

x = pi/2;
g_start = [cos(x) -sin(x) 0 .25;
           -sin(x) cos(x) 0 .6;
           0 0 -1 .22;
           0 0 0 1];

g_end = [cos(x) -sin(x) 0 .4;
        -sin(x) cos(x) 0 .45;
         0 0 -1 .22;
         0 0 0 1];
pen_tip_offset = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];
pen_tip_offset_inv = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1]; %inverse pen tip transformation from tool tip to base_link
theta_start = ur5InvKin(g_start * pen_tip_offset_inv);
theta_end = ur5InvKin(g_end * pen_tip_offset_inv);

ur5 = ur5_interface();

% %start position:
% start=input("Record start position");
% theta_start = ur5.get_current_joints();
% % end position:
% endGoal = input("Record goal position");
% theta_end = ur5.get_current_joints();

% choose control type
cType = input("Choose mode: 1 -> Inv Kin Control, 2 -> RR Control, 3 -> Trans Jac Control");
while (cType ~= 1 && cType ~= 2 && cType ~= 3)
    cType = input("Invalid input:1 -> Inv Kin Control, 2 -> RR Control, 3 -> Trans Control");
end
if cType == 1
    IKControlFunc(ur5, theta_start, theta_end);
elseif cType == 2
        RRControlFunc(ur5, theta_start, theta_end);
elseif cType == 3
        TransJacobianFunc(ur5, theta_start, theta_end);
end

