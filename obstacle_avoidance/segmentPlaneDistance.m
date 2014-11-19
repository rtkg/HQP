function [P d n]=lineSegmentPlaneDistance(L,A,b)
%Calculates the point with the largest (in a positive sense) distance of a plane and a line segment given in the 2 columns of L
%The plane is given by Ax-b=0. If the closest point lies in Ax-b >= 0, the distance d will be negatinve

%P ... closest point  (empty if parallel)
%d ... distance
%n ... normal pointing from the plane to the line

tol=1e-5;
A=A(:);

d1=pointPlaneDistance(L(:,1),A,b);
d2=pointPlaneDistance(L(:,2),A,b);

n=-A/norm(A);

if (abs(d2-d1) < tol)
    %parallel
    P=[];
    d=d1;
    return;
end    

if (d2 > d1)
    d=d2;
    P=L(:,2);
else
    d=d1;
    P=L(:,1);
end    
