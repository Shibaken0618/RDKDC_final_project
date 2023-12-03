function J = ur5BodyJacobian(q)
    %Joint Distances (mm)
    L0 = 89.2;
    L1 = 425;
    L2 = 392;
    L3 = 109.3;
    L4 = 94.75;
    L5 = 82.5;
    %Twists
    gst0 = [1,0,0,0
            0,0,-1,L3 + L5
            0,1,0, L0 + L1 + L2 + L4
            0,0,0,1];
    screw1=[cos(q(1)),-sin(q(1)),0,0
            sin(q(1)),cos(q(1)),0,0
            0,0,1,0
            0,0,0,1];
    screw2=[cos(q(2)),0,sin(q(2)),L0
            0,1,0,0
            -sin(q(2)),0,cos(q(2)),L0 - L0*cos(q(2))
            0,0,0,1];
    screw3=[cos(q(3)),0,sin(q(3)),-sin(q(3))*L0
            0,1,0,0
            -sin(q(2)),0,cos(q(2)),L0 - L0*cos(q(2))
            0,0,0,1];
    screw4= 
    screw5= 
    screw6= 

end