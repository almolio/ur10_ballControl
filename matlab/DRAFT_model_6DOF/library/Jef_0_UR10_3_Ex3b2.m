% This function outputs the system's Jacobian
%The input vector is:
%u=[q1 q2 q3 qp1 qp2 qp3 L1 L2 L3 L4 L5 L6]
function Jef_0=Jef_0_UR10_3(u)

q1=u(1);
q2=u(2);
q3=u(3);
L1=u(7);
L2=u(8);
L3=u(9);
L4=u(10);
L5=u(11);
L6=u(12);

Jef_0=  [L3*cos(q1) + L6*cos(q1) + L2*sin(q1)*sin(q2) - cos(q2)*sin(q1)*sin(q3)*(L4 + L5) + cos(q3)*sin(q1)*sin(q2)*(L4 + L5), -cos(q1)*(L2*cos(q2) + L4*cos(q2 - q3) + L5*cos(q2 - q3)), cos(q2 - q3)*cos(q1)*(L4 + L5); L3*sin(q1) + L6*sin(q1) - L2*cos(q1)*sin(q2) + cos(q1)*cos(q2)*sin(q3)*(L4 + L5) - cos(q1)*cos(q3)*sin(q2)*(L4 + L5), -sin(q1)*(L2*cos(q2) + L4*cos(q2 - q3) + L5*cos(q2 - q3)), cos(q2 - q3)*sin(q1)*(L4 + L5); 0, - L2*sin(q2) - L4*sin(q2 - q3) - L5*sin(q2 - q3), sin(q2 - q3)*(L4 + L5); 0, sin(q1), -sin(q1); 0, -cos(q1), cos(q1); 1, 0, 0];
