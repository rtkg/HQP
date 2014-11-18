function T=testBedGetTransform(q)
q=q(:);

T=eye(4);
T(1:3,4)=q(3:5);
T(1:3,1:3)=rz(q(2))*ry(q(1));

  