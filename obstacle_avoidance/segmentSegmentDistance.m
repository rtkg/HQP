function [P n d l]=segmentSegmentDistance(L1,L2)
%
%Calculates the points P=[P1 P2] of the line segments (given as the 2 columns in L1, L2) where the
%two segments are the closest. the distance is d, l=[l1 l2] \in [0 1] give the linear combination
%parameters and n is the unit vector pointing from P2 to P1. l is empty if the segments
%are parallel and partially overlapping (no unique solution)

tol=1e-5;

u=L1(:,2)-L1(:,1);
v=L2(:,2)-L2(:,1);

W=(L1(:,1)-L2(:,1));
U=[u -v];

H=U'*U;
f=U'*W;
lb=zeros(2,1);
ub=ones(2,1);

[l,lambda,status] =qld(H,zeros(1,2),f,0,lb,ub,0,1);
if (status~=0)
    error('Problem solving the QP in segmentSegmentDistance!');
end

P(:,1)=L1(:,1)+l(1)*u;
P(:,2)=L2(:,1)+l(2)*v;

n=P(:,1)-P(:,2); 
d=norm(n);
n=n/d; %normalize

%check if partially overlapping and parallel (no unique solution)
if (norm(lambda) < tol)
 P=[]; l=[];    
end

