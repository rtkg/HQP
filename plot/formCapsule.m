function C=formCapsule(r,l,plot_settings)
    
    N=20; %discretization points (should be even)
    [x,y,z] = sphere(N);
    
    if nargin < 2
        C.geom(1).x=x*r; C.geom(1).y=y*r; C.geom(1).z=z*r;
        C.plot_settings.col='y';
        C.plot_settings.alpha=1;
        return;
    elseif nargin < 3
        C.plot_settings.col='y';
        C.plot_settings.alpha=1;
        
    else
        C.plot_settings=plot_settings;
    end
 
    if isempty(l)
        return;
    end
    
    %Make a capsule
    a=linspace(0,r,N);
    
    b=sqrt(r^2-a.*a);
    t=[fliplr(b) ones(1,ceil(N*l/r))*r b]; 

    %    keyboard
    [x y z]=cylinder(t);
    C.geom(1).x=x;
    C.geom(1).y=y;
    C.geom(1).z=z*(l+2*r)-r;