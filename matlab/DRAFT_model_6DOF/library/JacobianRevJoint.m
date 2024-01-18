% input1: T(i-1)_0 previous joint ID i-1 wrt base 0
% input2: Tn_0 transform to end effector wrt base 0
function Ji=JacobianRevJoint(Ti_1_0, Tn_0)
    Ji = sym(zeros(6,1));
    tef_0 = sym(zeros(3,1));
    ti_1_0 = sym(zeros(3,1));
    zi_1 = sym(zeros(3,1));
    ti_1_0 = sym(zeros(3,1));
    vel_linear = sym(zeros(3,1));
    
    tef_0 = Tn_0(1:3,4);
    ti_1_0 = Ti_1_0(1:3,4);
    zi_1 = Ti_1_0(1:3,3);
    vel_linear = cross(zi_1, (tef_0-ti_1_0), 1);
    Ji(1:6,1) = cat(1, vel_linear, zi_1);
end