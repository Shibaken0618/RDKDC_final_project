ur5 = ur5_interface();
start = [pi/12;-pi/2;pi/12;-pi/2;pi/12;pi/12];
ur5.move_joints(start,10);
pause(10)

thetas = [pi/4;-pi/4;pi/12;-pi/2;pi/12;pi/12];
gdesired = ur5FwdKin(thetas);
%Frame_Desired = tf_frame('base_link','Goal', gdesired);
%pause(.5)(norm(w_k-w_final))

error = ur5RRcontrol_sam(gdesired,.05,ur5)

