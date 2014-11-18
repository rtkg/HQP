clear all; close all; clc;

addpath('../plot');

A=-rand(3,1); A=A/norm(A);
b=1;
P=rand(3,1);
r=1;

plot_hyperplane_HK(A,b,r,'m',0.5,0,1); hold on;
plot3(P(1),P(2),P(3),'b*');

axis equal; axis vis3d; rotate3d on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');

d=pointPlaneDistance(P,A,b);

