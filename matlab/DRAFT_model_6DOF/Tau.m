function [ tau ] = Tau( u )

%TAU60
% This function generates a time-dependent control scheme for the robot
% input:
% desired position as specified in Simulink and simulated time
% output:
% vectors of desired positions (Q) and velocities (Qp)
% Times are defined in TrajGen.m file

t0 = 10;
t1 = 3; % onset PD control
t2 = t1 + t0; % onset PD with gravity compensation
t3 = t1 + 2*t0; % onset PID control
t4 = t1 + 3*t0; % onset time-dependent position
t5 = 200*t0; % time-dependent position with w2
t6 = 200*t0;
t=u(1);

disp("t")
disp(t)

q1d=u(2);
q2d=u(3);
q3d=u(4);
q1dp=u(5);
q2dp=u(6);
q3dp=u(7);

Kd=diag([u(8);u(9);u(10)]);
Kp=diag([u(11);u(12);u(13)]);

q1=u(14);
q2=u(15);
q3=u(16);
q1p=u(17);
q2p=u(18);
q3p=u(19);

Q=[q1;q2;q3];
Qp=[q1p;q2p;q3p];

Qd=[q1d;q2d;q3d];
Qdp=[q1dp;q2dp;q3dp];

%Joint Errors

DeltaQ = ((Q - Qd));
DeltaQp = (Qp - Qdp);

% disp("error on q")
% disp(DeltaQ)

DelataQ_int = [u(40);u(41);u(42)];

%Robot Parameters
m1=u(20);
m2=u(21);
m3=u(22);
g=u(23);

%TO CORRECT
L1=u(24);
L2=u(25);
L4=u(26);
L6=u(27);
L7=u(28);
L9=u(29);
L3=u(30);
L5=u(31);
L8=u(32);
L10=u(33);

Kis=diag([u(34);u(35);u(36)]);
Ki=diag([0;0;0]);

gx=u(37);
gy=u(38);
gz=u(39);

%% Define controllers PD, PD+G, PID
tauc = [0;0;0];
G = [0;0;0];
%PD / PD + G

if (t>t1 && t<=t2)
%     disp("PD control")
    tauc = -Kp * DeltaQ - Kd * DeltaQp; %
end

if (t>=t2 && t<=t3)
    G= [- g*gy*(m2*(L7*sin(q1) - L8*cos(q1)*cos(q2)) + m3*(L7*sin(q1) + sin(q1)*(L9 - L7 + 7/100) - L3*cos(q1)*cos(q2) - 10*cos(q1)*cos(q2)*cos(q3) + 10*cos(q1)*sin(q2)*sin(q3))) - g*gx*(m2*(L7*cos(q1) + L8*cos(q2)*sin(q1)) + m3*(L7*cos(q1) + cos(q1)*(L9 - L7 + 7/100) - 10*sin(q1)*sin(q2)*sin(q3) + L3*cos(q2)*sin(q1) + 10*cos(q2)*cos(q3)*sin(q1))); - g*gz*(10*m3*cos(q2 + q3) + L3*m3*cos(q2) + L8*m2*cos(q2)) - g*gx*cos(q1)*(10*m3*sin(q2 + q3) + L3*m3*sin(q2) + L8*m2*sin(q2)) - g*gy*sin(q1)*(10*m3*sin(q2 + q3) + L3*m3*sin(q2) + L8*m2*sin(q2)); -10*g*m3*(gz*cos(q2 + q3) + gx*sin(q2 + q3)*cos(q1) + gy*sin(q2 + q3)*sin(q1))];
    tauc = -Kp * DeltaQ - Kd * DeltaQp + G; %
%     disp("with gravity compensation")
end

%PID
if (t>=t3)
%     disp("PID control scheme")
    tauc = -Kp * DeltaQ - Kd * DeltaQp - Kis * DelataQ_int;
end

tau=[tauc;DeltaQ;DeltaQp];

% disp("desired position")
% disp(Qd')
% disp("desired velocity")
% disp(Qdp')
% disp("tauc")
% disp(tauc')
% 
% disp("compensation G")
% disp(G')

end

