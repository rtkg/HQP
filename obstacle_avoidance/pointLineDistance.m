function [P n d l]=pointLineDistance(P1,L)
%Calculates the closest distance of line L and a point given in P1, P=[P1 P2] gives the original
%point and the closest point on the line respectively. n is the normalized vector pointing from
%P2 to P1, d is the distance
%
%P ... [P1 P2]
%d ... distance
%n ... normal
%l ... distance from P11 to P2

P11=L(:,1);
u_=L(:,2)-P11; u_=u_/norm(u_);

P_=P11-P1;
l=(-P_'*u_);
P2=l*u_+P11;
n=P1-P2; d=norm(n); n=n/d;

P=[P1 P2];

% %DEBUG: Plotting
% close all;
% figure
% plot3(L(1,:),L(2,:),L(3,:),'k'); grid on; hold on;
% plot3(P1(1),P1(2),P1(3),'rs');
% plot3(P2(1),P2(2),P2(3),'bs');
% axis equal; axis vis3d; rotate3d on;
% keyboard
