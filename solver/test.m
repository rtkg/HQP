clear all; close all; clc;

% HQP(1).nx=3;
% HQP(1).Eq.A=rand(1,3); HQP(1).Eq.A=[0.7770    0.5019    0.4255];
% HQP(1).Eq.b=rand(1,1); HQP(1).Eq.b=0.6112;
% HQP(1).IEq.A=[];
% HQP(1).IEq.b=zeros(0,1);

% HQP(2).Eq.A=rand(1,3); HQP(2).IEq.A=[0.8558    0.6708    0.5236];
% HQP(2).Eq.b=rand(1,1); HQP(2).IEq.b=0.2988;
% HQP(2).Eq.A=[];
% HQP(2).Eq.b=zeros(0,1);

% HQP(3).Eq.A=rand(1,3); HQP(3).Eq.A=[0.7040    0.3816    0.5677];
% HQP(3).Eq.b=rand(1,1); HQP(3).Eq.b=0.8879;
% HQP(3).IEq.A=[];
% HQP(3).IEq.b=zeros(0,1);

load IHQP.mat; HQP=IHQP;
tic
HQP=solveHQP(HQP);
toc

r=4;
shade=0.5;
t_o=0.1;
e_cols=['r' 'g' 'b'];
ie_cols=['m' 'y' 'c'];

plot3(0,0,0,'k+'); hold on;
for k=1:length(HQP)
    %plot equalities
    for j=1:length(HQP(k).Eq.b)
    plot_hyperplane_HK(HQP(k).Eq.A(j,:)', HQP(k).Eq.b,r,e_cols(k),shade,0);
    end
    %plot inequalities
    for j=1:length(HQP(k).IEq.b)
        plot_hyperplane_HK(HQP(k).IEq.A(j,:)',HQP(k).IEq.b,r,ie_cols(k),shade,0,1);
    end
    if k==1
        xK_P=[0; 0; 0];
    else
       xK_P=HQP(k-1).x;
    end
    
    plot3( HQP(k).x(1), HQP(k).x(2), HQP(k).x(3),'ko','MarkerFaceColor','k','MarkerSize',5);
    text( HQP(k).x(1)+t_o, HQP(k).x(2)+t_o, HQP(k).x(3)+t_o,strcat('x_',num2str(k)));
   % plot3([xK_P(1) HQP(k).x(1)],[xK_P(2) HQP(k).x(2)],[xK_P(3) HQP(k).x(3)],'k');
   % plot3( HQP(k).x(1), HQP(k).x(2), HQP(k).x(3),'ko','MarkerFaceColor','k','MarkerSize',5);
end    
grid on; axis vis3d; axis equal; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
view(-47.5,42);
%P1=eye(3)-A1'*inv(A1*A1')*A1 nullspace projector (A=row vector)

%x2=pinv(A1)*b1+pinv(A2*P1)*(b2-A2*pinv(A1)*b1) second-level pseudo inverse solution