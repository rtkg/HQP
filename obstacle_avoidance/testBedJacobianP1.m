function J=testBedJacobianP1(r,q)


J=testBedJacobianC;

th=q(1); ph=q(2);
J(1:3,1:2)=[-r*cos(th)*cos(ph)  r*sin(th)*sin(ph);
            -r*cos(th)*sin(ph) -r*sin(th)*cos(ph);
             r*sin(th)                 0          ];






