function D=computeObstacleConstraintParameters(O1,O2,q)
q=q(:);

ds=max(O1.ds,O2.ds);
di=max(O1.di,O2.di);

if ((strcmp(O1.type,'capsule') & strcmp(O2.type,'plane')) | (strcmp(O2.type,'capsule') & strcmp(O1.type,'plane')))    %capsule - plane

    if (strcmp(O1.type,'capsule'))
       r=O1.r; L=O1.L;
       A=O2.A; b=O2.b;
    else
        r=O2.r; L=O2.L;
        A=O1.A; b=O1.b;
    end
    
    %Get the end points of the line segment describing the capsule
    D(1).P=testBedForwardKinematics(0,q);
    D(2).P=testBedForwardKinematics(L,q);
    D(1).l=0; D(2).l=L; %distances for the Jacobain
    
    %Distance between the line segment points
    [d1 n]=pointPlaneDistance(D(1).P,A,b); 
    [d2 n]=pointPlaneDistance(D(2).P,A,b);
    
    if((d1 > 0) | (d2 > 0))
        warning('Attenzione: line segment is penetrating');
    end
    
    D(1).n=n; D(2).n=n;
    D(1).d=d1; D(2).d=d2;
  
elseif (strcmp(O1.type,'capsule') & strcmp(O2.type,'capsule'))    %capsule - capsule

    
else
      error(char(strcat('Obstacle distance',{' '},O1.type,'-',O2.type,{' '},'not implemented.')));  
end    




