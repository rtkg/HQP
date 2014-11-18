function [P1 P2 d]=lineLineDistance(L1,L2)
%Calculates closest points and distance of the lines defined by the point-pairs forming the columns of L1,L2
%(see: http://geomalgorithms.com/a07-_distance.html)
%
%P1 ... closest point on L1 (empty if parallel)
%P2 ... closest point on L2 (empty if parallel)
%d ... distance

tol=1e-5;

w0=L1(:,1)-L2(:,1);
u=L1(:,2)-L1(:,1); v=L2(:,2)-L2(:,1);
a=u'*u; b=u'*v; c=v'*v; d=u'*w0; e=v'*w0;

if ((a*c-b^2) < tol)
    %lines are parallel  
    s2=d/b;
    d=norm(L1(:,1)-L2(:,1)-s2*v);
    P1=[]; P2=[]; 
    return
end

s1=(b*e-c*d)/(a*c-b^2); s2=(a*e-b*d)/(a*c-b^2);
P1=L1(:,1)+s1*u;
P2=L2(:,1)+s2*v;
