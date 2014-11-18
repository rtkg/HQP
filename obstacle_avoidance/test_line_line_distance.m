clear all; close all; clc;

L1=rand(3,2); 
L2=rand(3,2);

plot3(L1(1,:),L1(2,:),L1(3,:),'b'); hold on;
plot3(L1(1,:),L1(2,:),L1(3,:),'bo','MarkerFaceColor','b','MarkerSize',5);
plot3(L2(1,:),L2(2,:),L2(3,:),'g'); 
plot3(L2(1,:),L2(2,:),L2(3,:),'go','MarkerFaceColor','g','MarkerSize',5);

[P1 P2 d]=lineLineDistance(L1,L2);

plot3(P1(1),P1(2),P1(3),'m*','MarkerSize',10);
plot3(P2(1),P2(2),P2(3),'m*','MarkerSize',10);
plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)],'m');

axis equal; axis vis3d; rotate3d on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');

