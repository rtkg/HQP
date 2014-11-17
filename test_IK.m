clear all; close all; clc;
%Simple IK test - go to a plane (2nd level hierarchy) while staying above another plane (1st level hierarchy)

addpath('./model'); addpath('./solver');

SP = model_LBRiiwa820_APPLE();
SV = System_Variables(SP);

d_time=1e-2;
tol=1e-4;
max_iter=1e4;
lmbd=2;
a=[1 -2 3]'; a=a/norm(a); %Target plane
a2=[0 0 -1]'; a2=a2/norm(a2); %Obstacle plane

b=0.9;
b2=-0.2;
r=2; col=['m' 'r']; shade=0.5;

SV = calc_pos(SP,SV); pJ=fk_j(SP,SV,1:7); [pE, RE]=fk_e(SP,SV,SP.bN,SP.bP);
h=drawSystem(pJ,pE,RE,SP,SV);
plot_hyperplane_HK(a,b,r,col(1),shade,0,0);
plot_hyperplane_HK(a2,b2,r,col(2),shade,0,1);

count=1;
Q=[]; DQ=[];
while (1)
    e=a'*pE-b;
    e2=a2'*pE-b2;
    if ((abs(e) < tol) || (count > max_iter))
        break;
    end    
   Je = calc_Je(SP,SV,SP.bN,SP.bP); %calculate the EE jacobian
      
   %Formulate the HQP 
   
   %1st level hierarchy -> stay above the obstacle plane
    HQP(1).nx=SP.n;
    HQP(1).IEq.A=a2'*Je(1:3,:); 
    HQP(1).IEq.b=-lmbd*e2;
    HQP(1).Eq.A=[];
    HQP(1).Eq.b=zeros(0,1);
    
    %2nd level hierarchy -> go to the target plane      
    HQP(2).Eq.A=a'*Je(1:3,:); 
    HQP(2).Eq.b=-lmbd*e;
    HQP(2).IEq.A=[];
    HQP(2).IEq.b=zeros(0,1);
    
    %solve and extract new joint velocities
    HQP=solveHQP(HQP);
    if (abs(HQP(1).IEq.w) > 1e-5)
        warning('Hierarchy infeasible on level 1');
        keyboard
    end
    
    SV.dq=HQP(2).x;
    SV = int_euler(SV,d_time,1); %Integrate a step forward

   %update the manipulator 
  SV = calc_pos(SP,SV); pJ=fk_j(SP,SV,1:7); [pE, RE]=fk_e(SP,SV,SP.bN,SP.bP);
  
  %Append for plotting
  
  P(count).pJ=pJ; P(count).pE=pE; P(count).RE=RE;  P(count).SV=SV;
  Q=[Q; SV.q']; DQ=[DQ; SV.dq'];
  count=count+1;
end    

for i=1:count-1
    drawSystem(P(i).pJ,P(i).pE,P(i).RE,SP,SV,h);
end    

t=linspace(0,(count-1)*d_time,count-1);
figure;
subplot(1,2,1);
plot(t,Q); grid on; 
xlabel('t'); ylabel('q');
subplot(1,2,2)
plot(t,DQ); grid on; 
xlabel('t'); ylabel('dq');
 





