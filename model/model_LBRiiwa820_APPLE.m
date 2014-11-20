function SP = model_LBRiiwa820_APPLE()
% 
% model of the KUKA LBR 7 DOF arm (tentatively)
%
% the mass and inertia parameters are arbitrary set
% the center of mass is assumed to co-inside with the input joint
%

% definition of system structure
% ----------------------------------------
SP.C = 0:8;
SP.n = length(SP.C)-1;
SP.mode = 1;

% definition of joints
% ----------------------------------------
SP.J(1).t    = [0.1          -0.25        0.45 ]';
SP.J(1).f    = [  0.0     0.0      0.0 ]';
SP.J(1).rpy  = [ 0         0        -pi/4]';
SP.J(1).type = 'R';

%SP.J(2).t    = [  0.1+0.1575*sqrt(2)/2 -0.25-0.1575*sqrt(2)/2    0.45 ]';
SP.J(2).t    = [ 0.1575    0            0]';
SP.J(2).f    = [  0.0     0.0      0.0 ]';
SP.J(2).rpy  = [ 0         pi/2       0]';
SP.J(2).type = 'R';

SP.J(3).t    = [  0.0    0.0    0.2025 ]';
SP.J(3).f    = [  0.0    0.0    0.0 ]';
SP.J(3).rpy  = [  pi/2    0.0   0 ]';
SP.J(3).type = 'R';

SP.J(4).t    = [  0.0     0.21     0.0 ]';
SP.J(4).f    = [  0.0     0.0      0.0 ]';
SP.J(4).rpy  = [  -pi/2     0.0     0 ]';
SP.J(4).type = 'R';

SP.J(5).t    = [  0.0    0.0       0.21 ]';
SP.J(5).f    = [  0.0     0.0      0.0 ]';
SP.J(5).rpy  = [  pi/2   0  0 ]';
SP.J(5).type = 'R';

SP.J(6).t    = [  0.0     0.2 0 ]';
SP.J(6).f    = [  0.0     0.0      0.0 ]';
SP.J(6).rpy  = [  -pi/2   0     0.0 ]';
SP.J(6).type = 'R';

SP.J(7).t    = [  0.0     0.0      0.2 ]';
SP.J(7).f    = [  0.0     0.0      0.0 ]';
SP.J(7).rpy  = [  pi/2    0      0.0 ]';
SP.J(7).type = 'R';

SP.J(8).t    = [  0.0     0.126      0.0 ]';
SP.J(8).f    = [  0.0     0.0      0.0 ]';
SP.J(8).rpy  = [  -pi/2    0      0.0 ]';
SP.J(8).type = 'R';

% definition of links
% ----------------------------------------
for iL = 1:SP.n+1
  SP.L(iL).m = 1;
  SP.L(iL).I = eye(3);
end

% definition of end-effectors
% ----------------------------------------
SP.bN = 9;
SP.bP = [0; 0; 0.05];

% orientation offset for the end-effector
SP.bR = rz(0);

%%%EOF
