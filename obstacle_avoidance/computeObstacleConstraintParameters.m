function D=computeObstacleConstraintParameters(O1,O2)

ds=max(O1.ds,O2.ds);
di=max(O1.di,O2.di);

if ((strcmp(O1.type,'capsule') & strcmp(O2.type,'plane')) | (strcmp(O2.type,'capsule') & strcmp(O1.type,'plane')))    %capsule - plane
                                                                                                                      %no relative motion of the plane for now ... FIXXXMEEE!!
    if (strcmp(O1.type,'capsule'))
        r=O1.r; L=O1.L;
        A=O2.A; b=O2.b;
        q=O1.q(:);
    else
        r=O2.r; L=O2.L;
        A=O1.A; b=O1.b;
        q=O2.q(:);
    end
    
    %Get the end points of the line segment describing the capsule
    D(1).P=testBedForwardKinematics(0,q);
    D(2).P=testBedForwardKinematics(L,q);
    D(1).l=0; D(2).l=L; %distances for the Jacobain
    
    %Distance between the line segment points
    [d1 n]=pointPlaneDistance(D(1).P,A,b); 
    [d2 n]=pointPlaneDistance(D(2).P,A,b);
    
    if((d1 < 0) | (d2 < 0))
        warning('Attenzione: line segment is penetrating');
    end
    
    D(1).n=n; D(2).n=n;
    D(1).d=d1; D(2).d=d2;
    
elseif (strcmp(O1.type,'capsule') & strcmp(O2.type,'capsule'))    %capsule - capsule

    q=O1.q(:); %no relative motion of capsule 2 for now ... FIXXXMEEE!!
    
    %find the planes of capsule 2
    P21=testBedForwardKinematics(0,O2.q);
    P22=testBedForwardKinematics(O2.L,O2.q);
    A=P22-P21; A=A/norm(A);
    b1=A'*P21; b2=A'*P22;
    
    %Extreme points of line segment 1
    P11=testBedForwardKinematics(0,O1.q);
    P12=testBedForwardKinematics(O1.L,O1.q);

    L1=[P11 P12]; L2=[P21 P22];

    if(linesCoplanar(L1,L2)==0)
        %if the segments are not coplanar, only the closest point needs to be constrained
         [P n d l]=segmentSegmentDistance(L1,L2);
         D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l;
    else
        %check the 3 intersection cases for coplanar segments
        'here'
        nL1=norm(L1(:,2)-L1(:,1));
        l1=intersectSegmentPlane(L1,A,b1);
        l2=intersectSegmentPlane(L1,A,b2);
        
        
        
        if ((l >= 0) & (l <=nL1))
            %intersects
            %TODO
            %1) constrain closest point on the new segment P1-intersection point 
            
            
        else
            % segment is in the first region -> constrain only closest point   
         [P n d l]=segmentSegmentDistance(L1,L2);
         D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l;
        end    
        
                l=intersectSegmentPlane(L1,A,b2)
        
        
        keyboard
    end    

else
    error(char(strcat('Obstacle distance',{' '},O1.type,'-',O2.type,{' '},'not implemented.')));  
end    




