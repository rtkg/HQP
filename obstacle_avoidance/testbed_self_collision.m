clear all; close all; clc;
%Simple IK test - go to a plane (2nd level hierarchy) while staying above another plane (1st level hierarchy)

addpath('../model'); addpath('../solver'); addpath('../plot');

SP = model_LBRiiwa820_APPLE();
SV = System_Variables(SP);

d_time=1e-2;


SV.q=[0;rand(7,1)];
SV = calc_pos(SP,SV); pJ=fk_j(SP,SV,1:8); [pE, RE]=fk_e(SP,SV,SP.bN,SP.bP);
%h=drawSystem(pJ,pE,RE,SP,SV);
light('Position',[-1 0 0],'Style','local');

Draw_System(SP, SV, SP.bN, SP.bP, 1:8, 1); axis equal; grid on; rotate3d on; axis vis3d;


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

fr=10; %frame count for plotting
for i=1:count-1
    if (~mod(i,fr))
        tic; 
        drawSystem(P(i).pJ,P(i).pE,P(i).RE,SP,SV,h);
        pt=d_time*fr-toc;
        if (pt < 0)
            warning('Cannot keep up with plotting - increase the frame count ');
        else    
            pause(pt);
        end
    end
end    

t=linspace(0,(count-1)*d_time,count-1);
figure;
subplot(1,2,1);
plot(t,Q); grid on; 
xlabel('t'); ylabel('q');
subplot(1,2,2)
plot(t,DQ); grid on; 
xlabel('t'); ylabel('dq');
 





