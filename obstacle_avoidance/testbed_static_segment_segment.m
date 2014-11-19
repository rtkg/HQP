clear all; close all; clc;

addpath('../plot');

d_time=0.01; %time step for simulation
tol=1e-3;
max_iter=1e2;
fr=1; %frame count for plotting

%First-level hierarchy task -> keep capsules away from each other
Tasks{1}.level=1;
Tasks{1}.type='obstacle_avoidance';
%==================Obstacle 1============================
Tasks{1}.O1.type='capsule';
Tasks{1}.O1.r=0.1;
Tasks{1}.O1.L=0.5;
Tasks{1}.O1.C=formCapsule(Tasks{1}.O1.r,Tasks{1}.O1.L);
Tasks{1}.O1.ds=0.12;
Tasks{1}.O1.di=0.2;
%==================Obstacle 2============================
Tasks{1}.O2.type='capsule';
Tasks{1}.O2.r=0.1;
Tasks{1}.O2.L=0.5;
Tasks{1}.O2.C=formCapsule(Tasks{1}.O2.r,Tasks{1}.O2.L);
Tasks{1}.O2.ds=0.12;
Tasks{1}.O2.di=0.2;
Tasks{1}.lmbd=1;

%Second-level hierarcy task -> move capsule frame center to target
Tasks{2}.level=2;
Tasks{2}.type='point_to_point';
Tasks{2}.t=[-Tasks{1}.O1.L/2;0;0]; %target vector
Tasks{2}.lmbd=1;

nx=5; %q=[th, ph, c_x, c_y, c_z]'
Tasks{1}.O1.q=[pi/2;0;-Tasks{1}.O1.L/2;0;0.8]; %initial configuration
Tasks{1}.O2.q=[pi/2;0;-Tasks{1}.O2.L/2;0;0.4]; %initial configuration

%=============Initial Plotting===========================================
Tasks{1}.O2.h=surf(Tasks{1}.O2.C.geom.x,Tasks{1}.O2.C.geom.y,Tasks{1}.O2.C.geom.z,'FaceColor',Tasks{1}.O2.C.plot_settings.col,'FaceAlpha', Tasks{1}.O2.C.plot_settings.alpha,'EdgeAlpha',0); hold on;
Tasks{1}.O2.C.T=hgtransform('Parent',gca);
Tasks{1}.O1.h=surf(Tasks{1}.O1.C.geom.x,Tasks{1}.O1.C.geom.y,Tasks{1}.O1.C.geom.z,'FaceColor',Tasks{1}.O1.C.plot_settings.col,'FaceAlpha', Tasks{1}.O1.C.plot_settings.alpha,'EdgeAlpha',0); hold on;
Tasks{1}.O1.C.T=hgtransform('Parent',gca);
set(Tasks{1}.O1.h,'Parent',Tasks{1}.O1.C.T);
set(Tasks{1}.O2.h,'Parent',Tasks{1}.O2.C.T);

%Form transforms and set then
set(Tasks{1}.O1.C.T,'Matrix',testBedGetTransform(Tasks{1}.O1.q));
set(Tasks{1}.O2.C.T,'Matrix',testBedGetTransform(Tasks{1}.O2.q));

axlims=[-2 2 -0.2 0.2 -0.2 1];
axis(axlims);
pbaspect([axlims(2)-axlims(1) axlims(4)-axlims(3) axlims(6)-axlims(5)]);
light('Position',[-1 -1 1],'Style','local');
axis vis3d; rotate3d on; view(35,16); grid on;
xlabel('x'); ylabel('y'); zlabel('z');

%find & plot the planes of capsule 2
P1=testBedForwardKinematics(0,Tasks{1}.O2.q);
P2=testBedForwardKinematics(Tasks{1}.O2.L,Tasks{1}.O2.q);
A=P2-P1; A=A/norm(A);
b1=A'*P1; b2=A'*P2;
plot_hyperplane_HK(A,b1,1,'m',0.5,0,1);
plot_hyperplane_HK(A,b2,1,'m',0.5,0,1);

%========================================================================

%==========================form HQP=====================================

q=Tasks{1}.O1.q;
qd=zeros(5,1);

Q=[]; QD=[]; E=[];
count=1;
while(1)
    tic
    %Compute the relevant task error
    e=norm(q(3:end)-Tasks{2}.t);
    E=[E; e];
    if ((e < tol) || (count > max_iter));
        break;
    end
    
    %Compute constraint points, distances and normals for the given Obstacle pair
    D=computeObstacleConstraintParameters(Tasks{1}.O1,Tasks{1}.O2);
    
    HQP(1).nx=nx;
    HQP(1).IEq.A=[-D(1).n'*testBedJacobian(D(1).l,q); -D(2).n'*testBedJacobian(D(2).l,q)];
    HQP(1).IEq.b=[Tasks{1}.lmbd*(-D(1).d-Tasks{1}.O1.ds)/(Tasks{1}.O1.di-Tasks{1}.O1.ds); Tasks{1}.lmbd*(-D(2).d-Tasks{1}.O2.ds)/(Tasks{1}.O2.di-Tasks{1}.O2.ds)];
    HQP(1).Eq.A=zeros(0,nx);
    HQP(1).Eq.b=zeros(0,1);

    %Task 2: desired motion
    HQP(2).Eq.A=testBedJacobian(0,q);
    HQP(2).Eq.b=-Tasks{2}.lmbd*(q(3:end)-Tasks{2}.t);
    HQP(2).IEq.A=zeros(0,nx);
    HQP(2).IEq.b=zeros(0,1);

    %Solve the HQP
    HQP=solveHQP(HQP);
    qd=HQP(end).x;
 
    Q=[Q; q']; QD=[QD; qd']; 
    
    %Forward recursion
    q=q+d_time*qd;
        
    % %In-loop plotting
    % R=testBedGetTransform(q);
    % set(Tasks{2}.O.C.T,'Matrix',R);
    % drawnow update;        
    % pt=d_time*fr-toc;
    % pause(pt); 
    
    count=count+1;    
end

for i=1:count-1
    if (~mod(i,fr))
        tic; 
        set(Tasks{1}.O2.C.T,'Matrix',testBedGetTransform(Q(i,:)));
        drawnow update;        
        pt=d_time*fr-toc;
        if (pt < 0)
            warning('Cannot keep up with plotting - increase the frame count ');
        else    
            pause(pt);
        end
    end
end 

t=linspace(0,count*d_time,count-1);
figure;
subplot(1,3,1);
plot(t,Q(:,3)); grid on;
xlabel('t'); ylabel('x');
subplot(1,3,2);
plot(t,Q(:,4)); grid on;
xlabel('t'); ylabel('y');
subplot(1,3,3);
plot(t,Q(:,5)); grid on;
xlabel('t'); ylabel('z');

figure;
subplot(1,2,1);
plot(t,Q(:,1)); grid on;
xlabel('t'); ylabel('Theta');
subplot(1,2,2);
plot(t,Q(:,2)); grid on;
xlabel('t'); ylabel('Phi');