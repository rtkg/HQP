function l=intersectSegmentPlane(L,A,b)
%
tol=1e-5;

P1=L(:,1); u=L(:,2)-L(:,1);

%Segment and plane are parallel
if (abs(u'*A) < tol)
    l=[];
    return;
end

l=(b-A'*P1)/(A'*u);