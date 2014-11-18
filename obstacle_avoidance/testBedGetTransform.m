function T=testBedGetTransform(c,th,ph)

T=eye(4);
T(1:3,4)=c(:);
T(1:3,1:3)=rz(ph)*ry(th);

