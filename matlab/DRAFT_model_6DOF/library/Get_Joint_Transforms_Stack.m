%This function stacks in one matrix a robot's cf

%inputs
%T0_W: transform from base to wcf
%u=[q1,q2,q3,L1,L2,L3,L4,L5,L6];

%ouput
%Ti_W: 4x4x(n+2) matrix of transforms where n is the number of joints. This
%stack includes the transform to wcf at index 1 and end effector at index n+2.

function Ti_W=Get_Joint_Transforms_Stack(u,T0_W)

q1=u(1);
q2=u(2);
q3=u(3);
L1=u(4);
L2=u(5);
L3=u(6);
L4=u(7);
L5=u(8);
L6=u(9);

% Homogeneous Transformations of each joint (symbolic)
T1_0=[cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, 0; 0, 0, 0, 1];
T2_1=[-sin(q2), cos(q2), 0, -L2*sin(q2); cos(q2), sin(q2), 0, L2*cos(q2); 0, 0, -1, L3; 0, 0, 0, 1];
T3_2=[cos(q3), -sin(q3), 0, cos(q3)*(L4 + L5); sin(q3), cos(q3), 0, sin(q3)*(L4 + L5); 0, 0, 1, -L6; 0, 0, 0, 1];
T2_0 = T1_0 * T2_1;
T3_0 = T2_0 * T3_2;

% Homogeneous Transformations of each joint (symbolic) wrt the world coordinate frame (wcf)
Ti_W(:,:,1)=T0_W;
Ti_W(:,:,2)=T0_W * T1_0;
Ti_W(:,:,3)=T0_W * T2_0;
Ti_W(:,:,4)=T0_W * T3_0;

end