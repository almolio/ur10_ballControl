% This script generates all symbolic equations and transforms related to
% the robot UR10 with 3 dofs + dynamics as defined in tutorial 5

syms gx gy gz g real
syms q1 q2 q3 q4 q5 q6 qp1 qp2 qp3 qp4 qp5 qp6 qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 real
syms L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 positive
syms I1xx I1yy I1zz real
syms I2xx I2yy I2zz real
syms I3xx I3yy I3zz real
syms I4xx I4yy I4zz real
syms I5xx I5yy I5zz real
syms I6xx I6yy I6zz real
syms m1 m2 m3 m4 m5 m6 I_m1 I_m2 I_m3 I_m4 I_m5 I_m6 real
syms C M G tau

I_m1 = [I1xx, 0, 0; 0, I1yy, 0; 0, 0, I1zz];
I_m2 = [I2xx, 0, 0; 0, I2yy, 0; 0, 0, I2zz];
I_m3 = [I3xx, 0, 0; 0, I3yy, 0; 0, 0, I3zz];
I_m4 = [I4xx, 0, 0; 0, I4yy, 0; 0, 0, I4zz];
I_m5 = [I5xx, 0, 0; 0, I5yy, 0; 0, 0, I5zz];
I_m6 = [I6xx, 0, 0; 0, I6yy, 0; 0, 0, I6zz];

%Joint Position Vector
Q=[q1; q2; q3; q4; q5; q6];

%Joint Velocity Vector
Qp=[qp1; qp2; qp3; qp4; qp5; qp6];

%Joint Acceleration Vector
Qpp=[qpp1; qpp2; qpp3; qpp4; qpp5; qpp6];

%% 3 DOF
%% DH tables
% DH table links
% DH_links =  [q1         L1              0      -pi/2; 
%             q2          L7              L3      0;
%             q3          L7-L2-L4+0.07   L5      0];

% DH table cms
% DH_cms =    [q1         L6              0       -pi/2; 
%             q2          L7              L8      0;
%             q3          -(L7-L9-0.07)   10     0];

%% 6 DOF
% tables based on manufacturer's datasheet
DH_links =  [q1         0.1273          0          pi/2; 
            q2          0               -0.612      0;
            q3          0               -0.5723     0;  
            q4          0.163941        0           pi/2;
            q5          0.1157          0           -pi/2;
            q6          0.0922          0           0];

% translations of centers of mass
cm_coor =   [[0.021, 0.000, 0.027]; [0.38, 0.000, 0.158]; [0.24, 0.000, 0.068];
            [0.000, 0.007, 0.018]; [0.000, 0.007, 0.018]; [0, 0, -0.026]];

%number of joints
n=6;

% world cf
T0_W=sym(eye(4));
T0_W(1:3,1:3)=RotZ(pi);

%% transforms
% equations for the links (RELATIVE)
T1_0=relativeTrans(DH_links(1,:));
T2_1=relativeTrans(DH_links(2,:));
T3_2=relativeTrans(DH_links(3,:));
T4_3=relativeTrans(DH_links(4,:));
T5_4=relativeTrans(DH_links(5,:));
T6_5=relativeTrans(DH_links(6,:)); % end effector

%equations for the cms (RELATIVE)
Tcm1_0=T1_0;
Tcm1_0(1:3,4)= cm_coor(1,:)';

Tcm2_1=T2_1;
Tcm2_1(1:3,4)=cm_coor(2,:)';

Tcm3_2=T3_2;
Tcm3_2(1:3,4)=cm_coor(3,:)';

Tcm4_3=T4_3;
Tcm4_3(1:3,4)=cm_coor(4,:)';

Tcm5_4=T5_4;
Tcm5_4(1:3,4)=cm_coor(5,:)';

Tcm6_5=T6_5;
Tcm6_5(1:3,4)=cm_coor(6,:)';

% stacks
%equations for the links (wrt BASE)
Ti_0 = sym(zeros(4,4,n+1));
Ti_0(:,:,1) = T0_W;
Ti_0(:,:,2) = T1_0;
Ti_0(:,:,3) = T1_0 * T2_1;
Ti_0(:,:,4) = Ti_0(:,:,3) * T3_2;
Ti_0(:,:,5) = Ti_0(:,:,4) * T4_3;
Ti_0(:,:,6) = Ti_0(:,:,5) * T5_4;
Ti_0(:,:,7) = Ti_0(:,:,6) * T6_5;

%equations for the links (wrt BASE)
Tcmi_0 = sym(zeros(4,4,n+1));
Tcmi_0(:,:,1)=T0_W;
Tcmi_0(:,:,2)=vpa(Tcm1_0, 4);
Tcmi_0(:,:,3)=T1_0 * Tcm2_1;
Tcmi_0(:,:,4)=Ti_0(:,:,3) * Tcm3_2;
Tcmi_0(:,:,5)=Ti_0(:,:,4) * Tcm4_3;
Tcmi_0(:,:,6)=Ti_0(:,:,5) * Tcm5_4;
Tcmi_0(:,:,7)=Ti_0(:,:,6) * Tcm6_5;

%rotation matrices
Rcm1_0= Tcmi_0(1:3,1:3,2);
Rcm2_0= Tcmi_0(1:3,1:3,3);
Rcm3_0= Tcmi_0(1:3,1:3,4);
Rcm4_0= Tcmi_0(1:3,1:3,5);
Rcm5_0= Tcmi_0(1:3,1:3,6);
Rcm6_0= Tcmi_0(1:3,1:3,7);

%% geometric Jacobian of cms
Jcm1_0 = sym(zeros(6,n));
Jcm1_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,2));

Jcm2_0 = sym(zeros(6,n));
Jcm2_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,3));
Jcm2_0(1:6,2) = JacobianRevJoint(T1_0, Tcmi_0(:,:,3));

Jcm3_0 = sym(zeros(6,n)); 
Jcm3_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,4));
Jcm3_0(1:6,2) = JacobianRevJoint(T1_0, Tcmi_0(:,:,4));
Jcm3_0(1:6,3) = JacobianRevJoint(Ti_0(:,:,3), Tcmi_0(:,:,4));

Jcm4_0 = sym(zeros(6,n)); 
Jcm4_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,5));
Jcm4_0(1:6,2) = JacobianRevJoint(T1_0, Tcmi_0(:,:,5));
Jcm4_0(1:6,3) = JacobianRevJoint(Ti_0(:,:,3), Tcmi_0(:,:,5));
Jcm4_0(1:6,4) = JacobianRevJoint(Ti_0(:,:,4), Tcmi_0(:,:,5));

Jcm5_0 = sym(zeros(6,n)); 
Jcm5_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,6));
Jcm5_0(1:6,2) = JacobianRevJoint(T1_0, Tcmi_0(:,:,6));
Jcm5_0(1:6,3) = JacobianRevJoint(Ti_0(:,:,3), Tcmi_0(:,:,6));
Jcm5_0(1:6,4) = JacobianRevJoint(Ti_0(:,:,4), Tcmi_0(:,:,6));
Jcm5_0(1:6,5) = JacobianRevJoint(Ti_0(:,:,5), Tcmi_0(:,:,6));

Jcm6_0 = sym(zeros(6,n)); 
Jcm6_0(1:6,1) = JacobianRevJoint(T0_W, Tcmi_0(:,:,7));
Jcm6_0(1:6,2) = JacobianRevJoint(T1_0, Tcmi_0(:,:,7));
Jcm6_0(1:6,3) = JacobianRevJoint(Ti_0(:,:,3), Tcmi_0(:,:,7));
Jcm6_0(1:6,4) = JacobianRevJoint(Ti_0(:,:,4), Tcmi_0(:,:,7));
Jcm6_0(1:6,5) = JacobianRevJoint(Ti_0(:,:,5), Tcmi_0(:,:,7));
Jcm6_0(1:6,6) = JacobianRevJoint(Ti_0(:,:,6), Tcmi_0(:,:,7));

%% inertia matrix M in symbolic form
M= m1*Jcm1_0(1:3,:)'*Jcm1_0(1:3,:) + Jcm1_0(4:6,:)'*Rcm1_0*I_m1*Rcm1_0'*Jcm1_0(4:6,:);
M= M + m2*Jcm2_0(1:3,:)'*Jcm2_0(1:3,:) + Jcm2_0(4:6,:)'*Rcm2_0*I_m2*Rcm2_0'*Jcm2_0(4:6,:);
M= M + m3*Jcm3_0(1:3,:)'*Jcm3_0(1:3,:) + Jcm3_0(4:6,:)'*Rcm3_0*I_m3*Rcm3_0'*Jcm3_0(4:6,:);
M= M + m4*Jcm4_0(1:3,:)'*Jcm4_0(1:3,:) + Jcm4_0(4:6,:)'*Rcm4_0*I_m4*Rcm4_0'*Jcm4_0(4:6,:);
M= M + m5*Jcm5_0(1:3,:)'*Jcm5_0(1:3,:) + Jcm5_0(4:6,:)'*Rcm5_0*I_m5*Rcm5_0'*Jcm5_0(4:6,:);
M= M + m6*Jcm6_0(1:3,:)'*Jcm6_0(1:3,:) + Jcm6_0(4:6,:)'*Rcm6_0*I_m6*Rcm6_0'*Jcm6_0(4:6,:);

% for i = 1:6
%     disp("*******line*******")
%     for j = 1:6
%         disp("**column**")
%         disp(char(simplify(M(i,j))))
%     end
% end

%% Coriolis and centrifugal matrix C in symbolic form
C = sym(zeros(n));
for k = 1:n
    for j=1:n
        for i=1:n
            C(k,j) = C(k,j) + 0.5*(diff(M(k,j),Q(i)) + diff(M(k,i),Q(j)) - diff(M(i,j),Q(k))) * Qp(i);
        end
    end
end

% disp("***********************printing C***********************")
% for i = 5:6
%     disp("*******line*******")
%     for j = 1:6
%         disp(char(simplify(C(i,j))))
%     end
% end

%% gravitational torques G vector
G = sym(zeros(n,1));
P = [gx, gy, gz] * g *  (m1 * Tcmi_0(1:3,4,2) + m2 * Tcmi_0(1:3,4,3) + m3 * Tcmi_0(1:3,4,4) + ...
    m4 * Tcmi_0(1:3,4,5) + m5 * Tcmi_0(1:3,4,6) + m6 * Tcmi_0(1:3,4,7));
% disp("***********************printing G***********************")
for j=1:n
    G(j,1) = diff(P,Q(j));
%     disp(G(j,1))
end
% 
% disp("***********************END***********************")

disp("T1_0")
disp(char(simplify(T1_0)))
disp("T2_1")
disp(char(simplify(T2_1)))
disp("T3_2")
disp(char(simplify(T3_2)))
disp("T4_3")
disp(char(simplify(T4_3)))
disp("T5_4")
disp(char(simplify(T5_4)))
disp("T6_5")
disp(char(simplify(T6_5)))

disp("Tcm1_0")
disp(char(simplify(Tcm1_0)))
disp("Tcm2_1")
disp(char(simplify(Tcm2_1)))
disp("Tcm3_2")
disp(char(simplify(Tcm3_2)))
disp("Tcm4_3")
disp(char(simplify(Tcm4_3)))
disp("Tcm5_4")
disp(char(simplify(Tcm5_4)))
disp("Tcm6_5")
disp(char(simplify(Tcm6_5)))

% tau = M*Qpp + C*Qp + G;

% disp("tau")
% disp(char(simplify(tau)))