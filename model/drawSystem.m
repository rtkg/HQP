function h=drawSystem(pJ,pE,RE,SP,SV,h);

    s=0.1; %scale for frames, joint axis indicators

    %==================prepare the X/Y/ZData to be plotted for each handle
    XData{1}=pJ(1,:); YData{1}=pJ(2,:); ZData{1}=pJ(3,:); %Draw joint markers                                                       
    XData{end+1}=[pJ(1,:) pE(1)]; YData{end+1}=[pJ(2,:) pE(2)]; ZData{end+1}=[pJ(3,:) pE(3)]; %Lines between joints and EE   

    [e1 e2 e3]=getFrameData(SV.L(1).p, SV.L(1).R,s); %get the base frame data and append it
    XData{end+1}=e1(:,1); YData{end+1}=e1(:,2); ZData{end+1}=e1(:,3);
    XData{end+1}=e2(:,1); YData{end+1}=e2(:,2); ZData{end+1}=e2(:,3);
    XData{end+1}=e3(:,1); YData{end+1}=e3(:,2); ZData{end+1}=e3(:,3);

    [e1 e2 e3]=getFrameData(pE,RE,s); %get the EE frame data and append it
    XData{end+1}=e1(:,1); YData{end+1}=e1(:,2); ZData{end+1}=e1(:,3);
    XData{end+1}=e2(:,1); YData{end+1}=e2(:,2); ZData{end+1}=e2(:,3);
    XData{end+1}=e3(:,1); YData{end+1}=e3(:,2); ZData{end+1}=e3(:,3);

    
    %HACK: add a line representing the base in negative z of the first joint (for the KUKA LBRiiwa820)  
    ax=SV.L(2).R(:,3);
    XData{end+1}=[pJ(1,1) pJ(1,1)-ax(1)*0.1575]; 
    YData{end+1}=[pJ(2,1) pJ(2,1)-ax(2)*0.1575]; 
    ZData{end+1}=[pJ(3,1) pJ(3,1)-ax(3)*0.1575]; 
    
    for iJ = 1:size(pJ,2) % iterate over all joint axis to be plotted
        ax=SV.L(iJ+1).R(:,3);
        XData{end+1}=[pJ(1,iJ)-ax(1)/3*s  pJ(1,iJ)+ax(1)/3*s];
        YData{end+1}=[pJ(2,iJ)-ax(2)/3*s  pJ(2,iJ)+ax(2)/3*s];
        ZData{end+1}=[pJ(3,iJ)-ax(3)/3*s  pJ(3,iJ)+ax(3)/3*s];
    end
    
    %=================either plot or update the handles with the new data
    %plot
    if nargin > 5
        %update then handles
        nh=length(XData);
        if(nh ~=length(h))
            error('Invalid number of handle members!');
        end
        
        for i=1:nh
            set(h(i),'XData',XData{i},'YData',YData{i},'ZData',ZData{i});
        end
    else
        %make a new plot starting with the joint markers
        h(1)=plot3(XData{1},YData{1},ZData{1},'ko','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',1); hold on;
        
        %Draw the lines between joints and EE
        h(end+1)=plot3(XData{2},YData{2},ZData{2},'k','LineWidth',2);
        
        %Draw the base coordinate frame                                                                         
        h(end+1)=plot3(XData{3},YData{3},ZData{3},'r','LineWidth',2);
        h(end+1)=plot3(XData{4},YData{4},ZData{4},'g','LineWidth',2);
        h(end+1)=plot3(XData{5},YData{5},ZData{5},'b','LineWidth',2);

        %Draw the EE frame
        h(end+1)=plot3(XData{6},YData{6},ZData{6},'r','LineWidth',2);
        h(end+1)=plot3(XData{7},YData{7},ZData{7},'g','LineWidth',2);
        h(end+1)=plot3(XData{8},YData{8},ZData{8},'b','LineWidth',2);
        
        %HACK: Draw line representing the base in negative z of the first joint (for the KUKA LBRiiwa820)  
        h(end+1)=plot3(XData{9},YData{9},ZData{9},'k','LineWidth',2);

        for iJ = 1:size(pJ,2) % iterate over all joint axis to be plotted
            h(end+1)=plot3(XData{9+iJ}, YData{9+iJ}, ZData{9+iJ},'r','LineWidth',5);   
        end

        axlims=[-0.2 1.9 -2 1.4 -0.1 1.8];
        axis(axlims);
        pbaspect([axlims(2)-axlims(1) axlims(4)-axlims(3) axlims(6)-axlims(5)]);
        xlabel('x'),ylabel('y'),zlabel('z'); 
        axis vis3d; rotate3d on; grid on;
        
    end
    drawnow;
