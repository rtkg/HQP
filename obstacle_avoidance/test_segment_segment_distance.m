clear all; close all; clc;

L1=rand(3,2); 
L2=rand(3,2); %L2(:,2)=L2(:,1)+L1(:,2)-L1(:,1);

plot3(L1(1,:),L1(2,:),L1(3,:),'b'); hold on;
plot3(L1(1,:),L1(2,:),L1(3,:),'bo','MarkerFaceColor','b','MarkerSize',5);
text(L1(1,2),L1(2,2),L1(3,2),'L12');
text(L1(1,1),L1(2,1),L1(3,1),'L11');

plot3(L2(1,:),L2(2,:),L2(3,:),'g'); 
plot3(L2(1,:),L2(2,:),L2(3,:),'go','MarkerFaceColor','g','MarkerSize',5);
text(L2(1,2),L2(2,2),L2(3,2),'L22');
text(L2(1,1),L2(2,1),L2(3,1),'L21');

[P n d l]=segmentSegmentDistance(L1,L2);
P1=P(:,1); P2=P(:,2);

plot3(P1(1),P1(2),P1(3),'m*','MarkerSize',10);
plot3(P2(1),P2(2),P2(3),'m*','MarkerSize',10);

plot3([P2(1) P2(1)+n(1)*d],[P2(2) P2(2)+n(2)*d],[P2(3) P2(3)+n(3)*d],'k','LineWidth',4);

axis equal; axis vis3d; rotate3d on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');

