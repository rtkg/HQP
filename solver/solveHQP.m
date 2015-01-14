function HQP=solveHQP(HQP)

P=length(HQP); %hierarchy size
nx=HQP(1).nx;

for k=1:P
    A_eq=[]; b_eq=[];
    A_ieq=[]; b_ieq=[];
    nk_e=length(HQP(k).Eq.b);
    nk_ie=length(HQP(k).IEq.b);
         
    for i=1:k-1
        %stack the appropriate quantities
       A_eq=[A_eq; [HQP(i).Eq.A zeros(size(HQP(i).Eq.A,1),nk_e+nk_ie)]];
       b_eq=[b_eq; HQP(i).Eq.b+HQP(i).Eq.w]; 
       A_ieq=[A_ieq; [HQP(i).IEq.A zeros(size(HQP(i).IEq.A,1),nk_e+nk_ie)]];
       b_ieq=[b_ieq; HQP(i).IEq.b+HQP(i).IEq.w]; 
    end    
 
    A_eq=[A_eq; [HQP(k).Eq.A  -eye(nk_e) zeros(nk_e,nk_ie)]];
    A_ieq=[A_ieq; [HQP(k).IEq.A zeros(nk_ie,nk_e) -eye(nk_ie)]];
    b_eq=[b_eq; HQP(k).Eq.b];
    b_ieq=[b_ieq; HQP(k).IEq.b];

    H=2*blkdiag(zeros(nx,nx),eye(nk_e+nk_ie)); %Hessian
    nE=length(b_eq);
    f=zeros(nx+nk_e+nk_ie,1);
    lb=repmat(-Inf,nx+nk_e+nk_ie,1);
    ub=repmat(Inf,nx+nk_e+nk_ie,1);
    keyboard
    %Ax+b>=0
    [zeta,lambda,status] =qld(H, -[A_eq; A_ieq], f, [b_eq; b_ieq],lb, ub, nE,1);

    if (status ~= 0)
        error('infeasible problem');
    end    
    
    HQP(k).x=zeta(1:nx);
    HQP(k).Eq.w=zeta(nx+1:nx+nk_e);
    HQP(k).IEq.w=zeta(nx+nk_e+1:nx+nk_e+nk_ie);
end    
