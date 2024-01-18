%This function generates the handle of an ellipsoid as per the
%manipulability metrics

%inputs
%T0_W: transform to wcf
%xef_W: position of end effector
%Jef_0_lin: linear Jacobian in base cf
%w_max: max manipulability index

function Plot_Ellipsoid(T0_W, xef_W, Jef_0_lin, w_max)

% Compute Ellipsoid: Principal Axes 
[U,S,V] = svd(Jef_0_lin);

% Generate an Ellispoid with given semi-axis lengths
[x, y, z] = ellipsoid(0,0,0,sqrt(S(1,1)),sqrt(S(2,2)),sqrt(S(3,3)),10);

% % Project the unitary ellispsoid using the principal axes
% get transformation
R=eye(4);
R(1:3,1:3)=U;
Tellipsoid_W=T0_W*R; % rotate the axis of the frame
Tellipsoid_W(1:3,4)=xef_W; %translate to end effector

% apply homogeneous transformation to the ellipsoid
n = size(x,1);
x_T = reshape(x,[1 n*n]);
y_T = reshape(y,[1 n*n]);
z_T = reshape(z,[1 n*n]);
mesh = Tellipsoid_W * [x_T; y_T; z_T; ones(1,n*n)];
x = reshape(mesh(1,:),[n n]);
y = reshape(mesh(2,:),[n n]);
z = reshape(mesh(3,:),[n n]);

%plot
surf(x,y,z)
alpha(0.3)
text(1000, 1000, 1000, strcat('w_max = ', num2str(fix(w_max))),'FontSize',14);