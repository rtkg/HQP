function P1=testBedForwardKinematicsP1(r,q)
q=q(:);

th=q(1); ph=q(2);
P1=q(3:5)-[r*sin(th)*cos(ph); r*sin(th)*sin(ph); r*cos(th)];
