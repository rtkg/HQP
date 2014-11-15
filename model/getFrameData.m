function [e1 e2 e3]=getFrameData(p,R,s);
%plot data for the framevectors - each column holds x/y/z data respectively

e1=[p(1) p(2) p(3); p(1)+R(1,1)*s p(2)+R(2,1)*s  p(3)+R(3,1)*s];
e2=[p(1) p(2) p(3); p(1)+R(1,2)*s p(2)+R(2,2)*s  p(3)+R(3,2)*s];
e3=[p(1) p(2) p(3); p(1)+R(1,3)*s p(2)+R(2,3)*s  p(3)+R(3,3)*s];
    


