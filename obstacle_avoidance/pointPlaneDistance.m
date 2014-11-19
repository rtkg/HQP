function [d n]=pointPlaneDistance(P,A,b)
%Calculates the closestdistance of a plane and a point given in P
%The plane is given by Ax-b=0. If the point lies in Ax-b <= 0, the distance d will be negative
%
%d ... distance

A=A(:); P=P(:);
b=b/norm(A); A=A/norm(A);
d=(A'*P-b)*(-1); %(Ax-b=0)

n=-A;
   
