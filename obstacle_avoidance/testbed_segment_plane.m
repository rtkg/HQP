clear all; close all; clc;

addpath('../plot');

r=0.1;
l=0.5;
C=formCapsule(r,l);

A=[0; 0; -1]; A=A/norm(A);
b=-0.2;

C.geom.z=C.geom.z-l/2; %put the reference frame in the center for now
%=============Initial Plotting===========================================
%plot_hyperplane_HK(A,b,1,'m',0.5,0,1); hold on;
h=surf(C.geom.x,C.geom.y,C.geom.z,'FaceColor',C.plot_settings.col,'FaceAlpha', C.plot_settings.alpha,'EdgeAlpha',0); hold on;
C.T=hgtransform('Parent',gca);
set(h,'Parent',C.T);
%Form a transform and set it
Ri=testBedGetTransform([0;0;0.6],pi/2,0);
set(C.T,'Matrix',Ri);
axlims=[-0.4 0.4 -0.2 0.2 -0.2 0.7];
axis(axlims);
pbaspect([axlims(2)-axlims(1) axlims(4)-axlims(3) axlims(6)-axlims(5)]);
light('Position',[-1 -1 1],'Style','local');
axis vis3d; rotate3d on; view(0,1);
xlabel('x'); ylabel('y'); zlabel('z');
%========================================================================






% A=rand(3,1); A=A/norm(A);
% b=1;
% L=rand(3,2);
% r=1;

% [P d]=lineSegmentPlaneDistance(L,A,b);

% plot_hyperplane_HK(A,b,r,'m',0.5,0,1); hold on;
% plot3(L(1,:),L(2,:),L(3,:),'b'); hold on;
% plot3(L(1,:),L(2,:),L(3,:),'bo','MarkerFaceColor','b','MarkerSize',5);
% plot3(P(1),P(2),P(3),'m*','MarkerSize',10);
% d
% axis equal; axis vis3d; rotate3d on; grid on;
% xlabel('x'); ylabel('y'); zlabel('z');

