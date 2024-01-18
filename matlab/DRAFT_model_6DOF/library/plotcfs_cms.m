function plotcfs_cms(HT, labels)
    % % Visualize the respective coordinate frames
    tOff=-15;
    aLength=50;

    grid on
    hold on

    axisX_0=[aLength;0;0];
    axisY_0=[0;aLength;0];
    axisZ_0=[0;0;aLength];
    O_0=[0;0;0];

    [~,~,n]=size(HT);

    for i=1:n
        % % Visualize the Origin Oi_0
        Oi_0(:,i)=HT(:,:,i)*[O_0;1];
        axisX1_0=HT(:,:,i)*[axisX_0;1];
        axisY1_0=HT(:,:,i)*[axisY_0;1];
        axisZ1_0=HT(:,:,i)*[axisZ_0;1];

        % Origin C.F.
        plot3(Oi_0(1,i),Oi_0(2,i),Oi_0(3,i), 'k .','MarkerSize',30)
        text_label = ['cm_' num2str(i)];
        if n==1
            text_label = ['X_ef'];
            tOff = 2* tOff;
        end
        text(Oi_0(1,i)+tOff,Oi_0(2,i)+tOff,Oi_0(3,i)+tOff, text_label);
        %Plot x-axis
        plot3([Oi_0(1,i);axisX1_0(1)],[Oi_0(2,i);axisX1_0(2)],[Oi_0(3,i);axisX1_0(3)],'r -', 'Linewidth',2)
        %Plot y-axis
        plot3([Oi_0(1,i);axisY1_0(1)],[Oi_0(2,i);axisY1_0(2)],[Oi_0(3,i);axisY1_0(3)],'g -', 'Linewidth',2)
        %Plot z-axis
        plot3([Oi_0(1,i);axisZ1_0(1)],[Oi_0(2,i);axisZ1_0(2)],[Oi_0(3,i);axisZ1_0(3)],'b -', 'Linewidth',2)
    end

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end