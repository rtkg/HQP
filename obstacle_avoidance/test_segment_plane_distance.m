clear all; close all; clc;

addpath('../plot');

A=rand(3,1); A=A/norm(A);
b=1;
L=rand(3,2);
r=1;

[P d]=lineSegmentPlaneDistance(L,A,b);

plot_hyperplane_HK(A,b,r,'m',0.5,0,1); hold on;
plot3(L(1,:),L(2,:),L(3,:),'b'); hold on;
plot3(L(1,:),L(2,:),L(3,:),'bo','MarkerFaceColor','b','MarkerSize',5);
plot3(P(1),P(2),P(3),'m*','MarkerSize',10);
d
axis equal; axis vis3d; rotate3d on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');

