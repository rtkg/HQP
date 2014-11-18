clear all; close all; clc;

r=0.2;
l=0.4;
C=formCapsule(r,l);

h=surf(C.geom.x,C.geom.y,C.geom.z,'FaceColor',C.plot_settings.col,'FaceAlpha', C.plot_settings.alpha,'EdgeAlpha',0); hold on;

%Create a hgtransform for the current axis and set it as a parent for the geometry handle
C.T=hgtransform('Parent',gca);
set(h,'Parent',C.T);

%Form a transform and set it
R=eye(4);
R(1:3,1:3)=ry(pi/2); R(1:3,4)=[-r;0;0];
set(C.T,'Matrix',R);


light('Position',[-1 -1 1],'Style','local');
axis auto; axis vis3d; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
