clear all; close all; clc;

SP = model_LBRiiwa820_APPLE();
SV = System_Variables(SP);

SV.q=zeros(size(SV.q)); SV = calc_pos(SP,SV); pJ=fk_j(SP,SV,1:7); [pE, RE]=fk_e(SP,SV,SP.bN,SP.bP);
h=drawSystem(pJ,pE,RE,SP,SV);

return;
t=linspace(0,100,100);
for i=1:100
SV.q(1)=sin(t(i));
SV.q(2)=cos(t(i));
SV = calc_pos(SP,SV); pJ=fk_j(SP,SV,1:7); [pE, RE]=fk_e(SP,SV,SP.bN,SP.bP);
drawSystem(pJ,pE,RE,SP,SV,h);

end


