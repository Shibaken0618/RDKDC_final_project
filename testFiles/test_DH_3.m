%% Initialize
ur5 = ur5_interface;
pen_tip_offset1 = [1 0 0 0; 0 1 0 -.049; 0 0 1 .12228; 0 0 0 1];  %% %% Coordinate of pen-tip in tool frame
pen_tip_offset2 = [1 0 0 0; 0 1 0 .049; 0 0 1 -.12228; 0 0 0 1];

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

%% Draw
ur5.swtich_to_ros_control()
g_start = ur5FwdKin_DH(angles_start);

pen_start = g_start * pen_tip_offset1;

t_step = 1;
[x, y] = extra(150);
x = x.*0.003;
y = (y - y(1)).*0.003;
points = size(x, 2);
count = 1;

for i = 1:points
    pen_extra{i} = pen_start; 
    pen_extra{i}(1, 4) = pen_start(1, 4) + x(i);
    pen_extra{i}(2, 4) = pen_start(2, 4) + y(i);

    q_current = ur5.get_current_joints();
    angles_mid1 = ur5InvKin(pen_extra{i} * pen_tip_offset2);
    [~, min_error_i] = min(vecnorm(angles_mid1 - q_current, 1));  %% Using joints data to find the closest matching kinematic configuration 
    angles_mid = angles_mid1(:,min_error_i);
    Frame_G = tf_frame('base_link', ['Frame_G_', num2str(count)], pen_extra{i});
    count = count+1;
    ur5.move_joints(angles_mid, t_step);
    pause(t_step)
end
