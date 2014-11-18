clear all; close all; clc;

addpath('../plot');

d_time=0.01; %time step for simulation
tol=1e-3;
max_iter=1e2;
fr=1; %frame count for plotting

%first hierarchy task -> stay above a plane
T1.A=[0; 0; -1]; T1.A=T1.A/norm(T1.A);
T1.b=-0.2;
T1.lmbd=1;
T1.ds=0.12;
T1.di=0.2;

%second hierarchy task -> bring capsule center to a desired point
T2.r=0.1;
T2.l=0.5;
T2.C=formCapsule(T2.r,T2.l);
T2.C.geom.z=T2.C.geom.z-T2.l/2; %put the reference frame in the center for now
T2.lmbd=1;
T2.t=[0;0;0]; %target vector

nx=5; %q=[th, ph, c_x, c_y, c_z]'
q_i=[pi/2;0;0;0;0.6]; %initial configuration

%=============Initial Plotting===========================================
T1.h=plot_hyperplane_HK(T1.A,T1.b,1,'m',0.5,0,1); hold on;
T2.h=surf(T2.C.geom.x,T2.C.geom.y,T2.C.geom.z,'FaceColor',T2.C.plot_settings.col,'FaceAlpha', T2.C.plot_settings.alpha,'EdgeAlpha',0); hold on;
T2.C.T=hgtransform('Parent',gca);
set(T2.h,'Parent',T2.C.T);
%Form a transform and set it
Ri=testBedGetTransform(q_i);
set(T2.C.T,'Matrix',Ri);
axlims=[-0.4 0.4 -0.2 0.2 -0.2 0.7];
axis(axlims);
pbaspect([axlims(2)-axlims(1) axlims(4)-axlims(3) axlims(6)-axlims(5)]);
light('Position',[-1 -1 1],'Style','local');
axis vis3d; rotate3d on; view(0,1); grid on;
xlabel('x'); ylabel('y'); zlabel('z');
%========================================================================

%==========================form HQP=====================================

q=q_i;
qd=zeros(5,1);
Q=[]; QD=[]; E=[];
count=1;
while(1)
    tic
    %Compute the relevant task error
    e=norm(q(3:end)-T2.t);
    E=[E; e];
    if ((e < tol) || (count > max_iter));
        break;
    end
    
    %Task 1: obstacle avoidance
    P1=testBedForwardKinematicsP1(T2.r,q);
    P2=testBedForwardKinematicsP2(T2.r,q);

    %Distance between the line segment points
    d1=pointPlaneDistance(P1,T1.A,T1.b); 
    d2=pointPlaneDistance(P2,T1.A,T1.b);
    if((d1 > 0) | (d2 > 0))
        warning('Attenzione: line segment is penetrating');
    end
    
    n=-T1.A/norm(T1.A);
    
    HQP(1).nx=nx;
    HQP(1).IEq.A=[-n'*testBedJacobianP1(T2.r,q); -n'*testBedJacobianP2(T2.r,q)];
    HQP(1).IEq.b=[T1.lmbd*(-d1-T1.ds)/(T1.di-T1.ds); T1.lmbd*(-d2-T1.ds)/(T1.di-T1.ds)];
    HQP(1).Eq.A=zeros(0,nx);
    HQP(1).Eq.b=zeros(0,1);

    %Task 2: desired motion
    HQP(2).Eq.A=testBedJacobianC;
    HQP(2).Eq.b=-T2.lmbd*(q(3:end)-T2.t);
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
    % set(T2.C.T,'Matrix',R);
    % drawnow update;        
    % pt=d_time*fr-toc;
    % pause(pt); 
    
    count=count+1;    
end

for i=1:count-1
    if (~mod(i,fr))
        tic; 
        set(T2.C.T,'Matrix',testBedGetTransform(Q(i,:)));
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