function P1=testBedForwardKinematics(l,q)
q=q(:);

th=q(1); ph=q(2);
P1=q(3:5)+[l*sin(th)*cos(ph); l*sin(th)*sin(ph); l*cos(th)];
