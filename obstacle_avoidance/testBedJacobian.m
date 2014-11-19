function J=testBedJacobian(l,q)

th=q(1); ph=q(2);
J(1:3,1:2)=[l*cos(th)*cos(ph)  -l*sin(th)*sin(ph);
            l*cos(th)*sin(ph)   l*sin(th)*cos(ph);
             -l*sin(th)                 0         ];

J(1:3,3:5)=eye(3);




