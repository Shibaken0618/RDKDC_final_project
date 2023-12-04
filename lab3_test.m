ur5 = ur5_interface();
start = [.1;-pi/2;.1;-pi/2;.1;.1];
%ur5.move_joints(start,10);

thetas = [pi/6;pi/6;pi/6;pi/6;pi/6;pi/6];

gdesired = ur5FwdKin(thetas);
gst = ur5FwdKin(thetas);
J = ur5BodyJacobian(thetas);

error = ur5RRcontrol_sam(gdesired,.1,ur5);

