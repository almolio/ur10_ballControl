function [qd] = TrajGen(v)

%TRAJGEN
% This function generates a time-dependent desired position for the robot
% input:
% desired position and time as specified in Simulink and simulated time
% output:
% vectors of desired positions (Q) and velocities (Qp)

t0 = 1;
t4 = 100; % onset time-dependent position
t5 = 700*t0; % time-dependent position with w2
t6 = 200*t0;

t=v(4);
% disp(t)

% Generate Different trajectories at different times
%% Regulation
if (t<=t4)
%     disp("constant position")
%     disp(t)
%     disp(t4)
    q1d=deg2rad(v(1));
    q2d=deg2rad(v(2));
    q3d=deg2rad(v(3));
    q1dp=0;
    q2dp=0;
    q3dp=0;
end

%% Tracking
A=pi/4;
phase = 0;
if (t>=t4 && t<=t5)
    w=2*pi*0.05;
end

q1dp=0;
q2dp=0;
q3dp=0;

if (t>=t4)
    A=pi/4;
    phase = 0;
    w=2*pi*0.05;
%     disp("time-dep position")
    q1d=A * sin(w * t);
    q2d=A * sin(w * t + phase);
    q3d=A * sin(w * t + 2*phase);
    q1dp=A * w * cos(w * t);
    q2dp=A * w * cos(w * t + phase);
    q3dp=A * w * cos(w * t + 2*phase);
end
% A=pi/4;
% phase = 0;
% w=2*pi*0.1;
% q1d=A * sin(w * t);
% q2d=A * sin(w * t);
% q3d=A * sin(w * t);
% q1dp=A * w * cos(w * t);
% q2dp=A * w * cos(w * t);
% q3dp=A * w * cos(w * t);
% 
Qd = [q1d;q2d;q3d];
Qdp = [q1dp;q2dp;q3dp];

% disp("position")
% disp(Qd)
% Qd = pi/4*[0;-1;-1];


qd=[Qd;Qdp];

end

