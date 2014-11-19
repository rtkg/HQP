function c=linesCoplanar(L1,L2)
%
tol=1e-5;

c=0;

u=L1(:,2)-L1(:,1);
v=L2(:,2)-L2(:,1);

assert(norm(u) > tol); assert(norm(v) > tol);

if (norm(cross(u,v)) < tol)
    %If lines are parallel they are coplanar for sure
    c=1;
    return;
end  
   
A=[u -v]; b=L2(:,1)-L1(:,1);
A_=rref([A b]); 
if (A_(end,end) < tol)
    c=1;
end

    