function p=isPointInRegion(P,reg,L);
P=P(:); 

A=L(:,2)-L(:,1); A=A/norm(A);
b1=A'*L(:,1); b2=A'*L(:,2);
A_=[A';A']; b_=[b1;b2];

X=A_*P-b_;
if (X(1) < 0)
    r=1;
elseif ((X(1) >= 0) & (X(2) <= 0))
    r=2;
elseif (X(2) > 0)
    r=3;
end

if (reg==r)
    p=1;
else
    p=0;
end
