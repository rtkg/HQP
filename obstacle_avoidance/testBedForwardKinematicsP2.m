function P2=testBedForwardKinematicsP2(l,q)
q=q(:);

th=q(1); ph=q(2);
P2=q(3:5)+ [l/2*sin(th)*cos(ph); l/2*sin(th)*sin(ph); l/2*cos(th)];
