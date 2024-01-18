function [ Xout ] = SimpleRobotPlotROS( u )

%SIMPLEROBOTPLOT
% This function publishes to ROS the joint states and Tf messages
% inputs:
% all robot and dynamics parameters and robot kinematics (Q, Qp)
% ouputs:
% messages published to ROS and position of end effector

persistent  jointpub jointmsg counter tftree tfStampedMsg tfStampedMsg1 tfStampedMsg2 tfStampedMsg3
persistent  tfStampedMsgcm1 tfStampedMsgcm2 tfStampedMsgcm3
persistent  joint_frames cm_frames

%Joint Position
q1=pi/8;
q2=-pi/8;
q3=pi/2;
q4=-pi/10;
q5=-pi/10;
q6=pi/2;
q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;

%Joint Velocity
qp1=0;
qp2=0;
qp3=0;
qp4=0;
qp5=0;
qp6=0;

%Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);
q5=u(5);
q6=u(6);

%Joint Velocity
qp1=u(7);
qp2=u(8);
qp3=u(9);
qp4=u(10);
qp5=u(11);
qp6=u(12);
id_offset=6;

%Gravity
g=u(id_offset+38);
%Time
t=u(id_offset+39);

% Robot Base
T0_W=eye(4);
T0_W(1:3,1:3)=RotZ(pi);

% Compute the Homogeneous Transformations
%% 3 DOF model
% T1_0=[cos(q1), 0, -sin(q1), 0; sin(q1), 0, cos(q1), 0; 0, -1, 0, L1; 0, 0, 0, 1];
% T2_1= [cos(q2), -sin(q2), 0, L3*cos(q2); sin(q2), cos(q2), 0, L3*sin(q2); 0, 0, 1, L7; 0, 0, 0, 1];
% T3_2=[cos(q3), -sin(q3), 0, L5*cos(q3); sin(q3), cos(Tcm3_2q3), 0, L5*sin(q3); 0, 0, 1, L7 - L4 - L2 + 7/100; 0, 0, 0, 1];
% Tcm1_0=[cos(q1), 0, -sin(q1), 0; sin(q1), 0, cos(q1), 0; 0, -1, 0, L6; 0, 0, 0, 1];
% Tcm2_1=[cos(q2), -sin(q2), 0, L8*cos(q2); sin(q2), cos(q2), 0, L8*sin(q2); 0, 0, 1, L7; 0, 0, 0, 1];
% Tcm3_2=[cos(q3), -sin(q3), 0, L10*cos(q3); sin(q3), cos(q3), 0, L10*sin(q3); 0, 0, 1, L9 - L7 + 7/100; 0, 0, 0, 1];

% T3_W = T0_W * T1_0 * T2_1 * T3_2;
% 
% Xef_W= T3_W(1:3,4);
% 
% % Stack of Transformations
% H_stack_joints(:,:,1) = T0_W;
% H_stack_joints(:,:,2) = T1_0;
% H_stack_joints(:,:,3) = T2_1;
% H_stack_joints(:,:,4) = T3_2;
% 
% H_stack_cms(:,:,1) = Tcm1_0;
% H_stack_cms(:,:,2) = Tcm2_1;
% H_stack_cms(:,:,3) = Tcm3_2;

%% 6 DOF model
T1_0 = [cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, 1273/10000; 0, 0, 0, 1];
T2_1 = [cos(q2), -sin(q2), 0, -(153*cos(q2))/250; sin(q2), cos(q2), 0, -(153*sin(q2))/250; 0, 0, 1, 0; 0, 0, 0, 1];
T3_2 = [cos(q3), -sin(q3), 0, -(5723*cos(q3))/10000; sin(q3), cos(q3), 0, -(5723*sin(q3))/10000; 0, 0, 1, 0; 0, 0, 0, 1];
T4_3 = [cos(q4), 0, sin(q4), 0; sin(q4), 0, -cos(q4), 0; 0, 1, 0, 1476649253021493/9007199254740992; 0, 0, 0, 1];
T5_4 = [cos(q5), 0, -sin(q5), 0; sin(q5), 0, cos(q5), 0; 0, -1, 0, 1157/10000; 0, 0, 0, 1];
T6_5 = [cos(q6), -sin(q6), 0, 0; sin(q6), cos(q6), 0, 0; 0, 0, 1, 461/5000; 0, 0, 0, 1];

Tcm1_0 = [cos(q1), 0, sin(q1), 21/1000; sin(q1), 0, -cos(q1), 0; 0, 1, 0, 27/1000; 0, 0, 0, 1]; 
Tcm2_1 = [cos(q2), -sin(q2), 0, 19/50; sin(q2), cos(q2), 0, 0; 0, 0, 1, 79/500; 0, 0, 0, 1];
Tcm3_2 = [cos(q3), -sin(q3), 0, 6/25; sin(q3), cos(q3), 0, 0; 0, 0, 1, 17/250; 0, 0, 0, 1];
Tcm4_3 = [cos(q4), 0, sin(q4), 0; sin(q4), 0, -cos(q4), 7/1000; 0, 1, 0, 9/500; 0, 0, 0, 1];
Tcm5_4 = [cos(q5), 0, -sin(q5), 0; sin(q5), 0, cos(q5), 7/1000; 0, -1, 0, 9/500; 0, 0, 0, 1];
Tcm6_5 = [cos(q6), -sin(q6), 0, 0; sin(q6), cos(q6), 0, 0; 0, 0, 1, -13/500; 0, 0, 0, 1];

% end effector position
T6_W = T0_W * T1_0 * T2_1 * T3_2 * T4_3 * T5_4 * T6_5;
Xef_W= T6_W(1:3,4);

% Stack of Transformations
H_stack_joints(:,:,1) = T0_W;
H_stack_joints(:,:,2) = T1_0;
H_stack_joints(:,:,3) = T2_1;
H_stack_joints(:,:,4) = T3_2;
H_stack_joints(:,:,5) = T4_3;
H_stack_joints(:,:,6) = T5_4;
H_stack_joints(:,:,7) = T6_5;

H_stack_cms(:,:,1) = Tcm1_0;
H_stack_cms(:,:,2) = Tcm2_1;
H_stack_cms(:,:,3) = Tcm3_2;
H_stack_cms(:,:,4) = Tcm4_3;
H_stack_cms(:,:,5) = Tcm5_4;
H_stack_cms(:,:,6) = Tcm6_5;

%% Initialize the publishers and messages

if t==0
    % TF publisher
    tftree = rostf;
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.Header.Stamp = rostime('now');
    joint_frames = ["T0_W","T1_0","T2_1","T3_2","T4_3","T5_4","T6_5"];
    cm_frames = ["cm1","cm2","cm3","cm4","cm5","cm6"];
    
    % Joint State Publisher
    %Use here the correct topic name --see bringup launch file--
    jointpub = rospublisher('/ursa_joint_states', 'sensor_msgs/JointState');
    jointmsg = rosmessage(jointpub);
    
    % specific names of the joints --see urdf file--
    jointmsg.Name={'ursa_shoulder_pan_joint', 'ursa_shoulder_lift_joint', 'ursa_elbow_joint', 'ursa_wrist_1_joint', 'ursa_wrist_2_joint', 'ursa_wrist_3_joint'};
    
    for i=1:6
        jointmsg.Velocity(i)=0.1;
        jointmsg.Effort(i)=0.1;
        jointmsg.Position(i)=0.0;
    end
    
    counter=0;
end

%% Publish the robot joints 

jointmsg.Header.Stamp=rostime('now');
jointmsg.Header.Seq=counter;
counter=counter+1;
jointmsg.Position=[q1, q2, q3, q4, q5, q6];
send(jointpub,jointmsg);
sampleTime=0.02;
if(~mod(t,sampleTime))
    %% Publish the tf's (Base, Links, CMs and EF), see the joint names generated by bringUR10.launch
    % publish the transforms of each of the robot's 4 joints
    % parent frame id
    tfStampedMsg.Header.Seq=counter;
    tfStampedMsg.Header.FrameId = 'world';
    
    % links
    for i_joint=1:7
        tfStampedMsg.Header.Stamp = jointmsg.Header.Stamp;
        tfStampedMsg.ChildFrameId = joint_frames(i_joint);
        position = H_stack_joints(1:3,4,i_joint);
        tfStampedMsg.Transform.Translation.X = position(1);
        tfStampedMsg.Transform.Translation.Y = position(2);
        tfStampedMsg.Transform.Translation.Z = position(3);
    
        q = rotm2quat(H_stack_joints(1:3,1:3,i_joint));
        tfStampedMsg.Transform.Rotation.W = q(1);
        tfStampedMsg.Transform.Rotation.X = q(2);
        tfStampedMsg.Transform.Rotation.Y = q(3);
        tfStampedMsg.Transform.Rotation.Z = q(4);
        sendTransform(tftree, tfStampedMsg);
        tfStampedMsg.Header.FrameId = joint_frames(i_joint);
    end
    
    % centers of mass
    for i_joint=1:6
        tfStampedMsg.Header.FrameId = joint_frames(i_joint+1);
        tfStampedMsg.Header.Stamp = jointmsg.Header.Stamp;
        tfStampedMsg.ChildFrameId = cm_frames(i_joint);
        position = H_stack_cms(1:3,4,i_joint);
        tfStampedMsg.Transform.Translation.X = position(1);
        tfStampedMsg.Transform.Translation.Y = position(2);
        tfStampedMsg.Transform.Translation.Z = position(3);
    
        q = rotm2quat(H_stack_cms(1:3,1:3,i_joint));
        tfStampedMsg.Transform.Rotation.W = q(1);
        tfStampedMsg.Transform.Rotation.X = q(2);
        tfStampedMsg.Transform.Rotation.Y = q(3);
        tfStampedMsg.Transform.Rotation.Z = q(4);
        sendTransform(tftree, tfStampedMsg);
    end
    
end
Xout=[Xef_W(1:3,1)];
end

