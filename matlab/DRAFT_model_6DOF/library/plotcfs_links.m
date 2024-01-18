%This function plots the robot's cf and links

%ouput
%HT: 4x4x(n+2) matrix of transforms where n is the number of joints. This
%stack includes the transform to wcf at index 1 and end effector at index n+2.

function plotcfs_links(HT)
        
    % % Visualize the respective coordinate frames
    tOff=0.030;
    aLength=0.150;

    grid on
    hold on

    axisX_0=[aLength;0;0];
    axisY_0=[0;aLength;0];
    axisZ_0=[0;0;aLength];

    % % Visualize the Origin O_0
    O_0=[0;0;0];
    plot3(O_0(1),O_0(2),O_0(3), 'k .','MarkerSize',30)
    text(O_0(1)+tOff,O_0(2)+tOff,O_0(3)+tOff, 'O_W','FontSize',16);
    %Plot x-axis
    plot3([O_0(1);axisX_0(1)],[O_0(2);axisX_0(2)],[O_0(3);axisX_0(3)],'r -', 'Linewidth',2)
    %Plot y-axis
    plot3([O_0(1);axisY_0(1)],[O_0(2);axisY_0(2)],[O_0(3);axisY_0(3)],'g -', 'Linewidth',2)
    %Plot z-axis
    plot3([O_0(1);axisZ_0(1)],[O_0(2);axisZ_0(2)],[O_0(3);axisZ_0(3)],'b -', 'Linewidth',2)

    [~,~,n]=size(HT);
    
    % plot each joint CF
    for i=1:n
        % % Visualize the Origin Oi_0
        Oi_0(:,i)=HT(:,:,i)*[O_0;1];
        axisX1_0=HT(:,:,i)*[axisX_0;1];
        axisY1_0=HT(:,:,i)*[axisY_0;1];                 
        axisZ1_0=HT(:,:,i)*[axisZ_0;1];

        % Origin C.F.
        plot3(Oi_0(1,i),Oi_0(2,i),Oi_0(3,i), 'k .','MarkerSize',30)
        text(Oi_0(1,i)+tOff,Oi_0(2,i)+tOff,Oi_0(3,i)+tOff, ['O_' num2str(i-1)]);
            
        %Plot x-axis
        plot3([Oi_0(1,i);axisX1_0(1)],[Oi_0(2,i);axisX1_0(2)],[Oi_0(3,i);axisX1_0(3)],'r -', 'Linewidth',3)
        %Plot y-axis
        plot3([Oi_0(1,i);axisY1_0(1)],[Oi_0(2,i);axisY1_0(2)],[Oi_0(3,i);axisY1_0(3)],'g -', 'Linewidth',3)
        %Plot z-axis
        plot3([Oi_0(1,i);axisZ1_0(1)],[Oi_0(2,i);axisZ1_0(2)],[Oi_0(3,i);axisZ1_0(3)],'b -', 'Linewidth',3)
    end

    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');         
    
    % Links:
    plot3([Oi_0(1,1);Oi_0(1,1)],[Oi_0(2,1);Oi_0(2,1)],[Oi_0(3,1);0],'k -', 'Linewidth',2)
    for i=1:n-1
        
        plot3([Oi_0(1,i);Oi_0(1,i+1)],[Oi_0(2,i);Oi_0(2,i+1)],[Oi_0(3,i);Oi_0(3,i+1)],'k -', 'Linewidth',2)
    end
end
