% input: T(i-1)_0 previous joint ID i-1 wrt base 0
function Ji=JacobianPrisJoint(Ti_1_0)
    Ji = sym(zeros(6,1));
    Ji(1:3,1) = Ti_1_0(1:3,3);
end