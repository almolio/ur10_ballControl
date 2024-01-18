%This function plots the manipulability index based on the robot's joint
%states

%inputs:
%q1, q2: nx1 values of joint states
%w: nx1 values of [0 1] normalized manipulability
%axis_labels: 1x3 strings of axis labels

function plot_qi_qi_w(q1,q2,w,axis_labels)

    figure('Name','Dependency between manipulability and joint states')
    tri = delaunay(q1,q2);
    plot(q1,q2,'.')
    
    %create mesh of triangles
    [r,c] = size(tri);

    %plot the mesh as surface
    trisurf(tri, q1, q2, w);
    xlabel(strcat(axis_labels(1), ' radian)'));
    ylabel(strcat(axis_labels(2), ' radian)'));
    zlabel(strcat(axis_labels(3), ' manipulability [0 1]'));

end