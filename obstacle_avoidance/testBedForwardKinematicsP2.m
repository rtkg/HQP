function P2=testBedForwardKinematicsP2(r,q)
q=q(:);

th=q(1); ph=q(2);
P2=q(3:5)+ [r*sin(th)*cos(ph); r*sin(th)*sin(ph); r*cos(th)];
