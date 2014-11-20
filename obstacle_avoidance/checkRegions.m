function D=checkRegions(L1,L2)
P1=L1(:,1); P2=L1(:,2); u=P2-P1;
A=L2(:,2)-L2(:,1); A=A/norm(A);
b1=A'*L2(:,1); b2=A'*L2(:,2);

if(isPointInRegion(P1,1,L2) && isPointInRegion(P2,1,L2))% P1 & P2 lie in region I
   %We only need to constrain the closest point and are done

    [P n d l]=segmentSegmentDistance(L1,L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l;
    return;
elseif (isPointInRegion(P1,1,L2) && isPointInRegion(P2,2,L2)) %P1 lies in I, P2 lies in II
    l1=intersectSegmentPlane(L1,A,b1); I1=P1+l1*u; %form intersection 1                                                

    %limit the closest point on P1-I1
    [P n d l]=segmentSegmentDistance([P1 I1],L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l;
    %limit the extreme points I1 & P2
    [P n d l]=pointLineDistance(I1,L2);
    D(2).n=n; D(2).P=P; D(2).d=d; D(2).l=l;
    [P n d l]=pointLineDistance(P2,L2);
    D(3).n=n; D(3).P=P; D(3).d=d; D(3).l=l;
    return;
elseif (isPointInRegion(P1,1,L2) && isPointInRegion(P2,3,L2)) %P1 lies in I, P2 lies in III
    l1=intersectSegmentPlane(L1,A,b1); I1=P1+l1*u; %form intersection 1                                                
    l2=intersectSegmentPlane(L1,A,b2); I2=P1+l2*u; %form intersection 2                                             
    [P n d l]=segmentSegmentDistance([P1 I1],L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l; 
    [P n d l]=segmentSegmentDistance([I2 P2],L2);
    D(2).n=n; D(2).P=P; D(2).d=d; D(2).l=l; 
    [P n d l]=pointLineDistance(I1,L2);
    D(3).n=n; D(3).P=P; D(3).d=d; D(3).l=l; 
    [P n d l]=pointLineDistance(I2,L2);
    D(4).n=n; D(4).P=P; D(4).d=d; D(4).l=l; 
    return;
elseif (isPointInRegion(P1,2,L2) && isPointInRegion(P2,2,L2)) %P1 and P2 lie in II
    [P n d l]=pointLineDistance(P1,L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l; 
    [P n d l]=pointLineDistance(P2,L2);
    D(2).n=n; D(2).P=P; D(2).d=d; D(2).l=l; 
    return;
elseif (isPointInRegion(P1,2,L2) && isPointInRegion(P2,3,L2)) %P1 lies in II, P2 lies in III
    l2=intersectSegmentPlane(L1,A,b2); I2=P1+l2*u; %form intersection 2
    [P n d l]=segmentSegmentDistance([I2 P2],L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l; 
    [P n d l]=pointLineDistance(P1,L2);
    D(2).n=n; D(2).P=P; D(2).d=d; D(2).l=l; 
    [P n d l]=pointLineDistance(I2,L2);
    D(3).n=n; D(3).P=P; D(3).d=d; D(3).l=l; 
    return;
elseif (isPointInRegion(P1,3,L2) && isPointInRegion(P2,3,L2)) % P1 & P2 lie in III
    [P n d l]=segmentSegmentDistance(L1,L2);
    D(1).n=n; D(1).P=P; D(1).d=d; D(1).l=l; 
    return;
else
    'Recursing'
    D=checkRegions([P2 P1],L2); %Do recursion with flipped points P1 & P2  
end    

