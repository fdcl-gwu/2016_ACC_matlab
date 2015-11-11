% 22 September 2015
% load constants for simulation

constants.avoid_switch = 'true';
constants.dist_switch = 'true';
constants.adaptive_switch = 'true';

% define constants/properties of rigid body
constants.m_sc = 1;
m_sc = constants.m_sc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INERTIA TENSOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% constants.J = [1.059e-2 -5.156e-6 2.361e-5;...
%                 -5.156e-6 1.059e-2 -1.026e-5;
%                 2.361e-5 -1.026e-5 1.005e-2];
% Chris's Hexrotor inertia matrix
constants.J = [55710.50413e-7 ,  617.6577e-7   , -250.2846e-7 ;...
               617.6577e-7    ,  55757.4605e-7 , 100.6760e-7 ;...
               -250.2846e-7  ,  100.6760e-7   , 105053.7595e-7];
                    
% % % from Farhad ASME paper
% constants.J = [ 5.5711 0.0618 -0.0251; ...
%                 0.06177 5.5757 0.0101;...
%                 -0.02502 0.01007 1.05053] * 1e-2;

% constants.J = diag([694 572 360]);

J = constants.J;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONTROLLER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% attitude controller parameters
constants.G = diag([0.9 1 1.1]);

constants.kp = 0.4;
constants.kw = 0.296; % 2*zeta*wn 

% position controller parameters
constants.alpha_pos = 5; % scale factor
constants.beta = 1; % std dev
constants.N = diag(constants.beta*ones(1,3));
constants.P = eye(3,3);
constants.det_shell = 1;

wn = 4;
zeta = 0.7;

constants.kx = constants.m_sc*2*zeta*wn;
constants.kv = constants.m_sc*wn^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONSTRAINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


constants.sen = [1;0;0]; % body fixed frame
% define a number of attitude constraints to avoids
% 
con = [0.174    0.4   -0.853 -0.122;...
    -0.934   0.7071    0.436 -0.140;...
    -0.034   0.7071   -0.286 -0.983];
constants.con_angle = [40;40;40;20]*pi/180;

% con = [1/sqrt(2);1/sqrt(2);0];
% constants.con_angle = 12*pi/180;

constants.con = con./repmat(sqrt(sum(con.^2,1)),3,1); % normalize

constants.alpha = 15; % use the same alpha for each one
constants.num_con = size(constants.con,2);

% position constraint
constants.xo = [5 ;...
      5;...
      0 ]; % obstacles are the column vectors
constants.num_obs = size(constants.xo,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ADAPTIVE CONTROL FOR DISTURBANCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disturbance terms

constants.W = eye(3,3);
constants.delta = [0.2;0.2;0.2];
constants.kd = 0.5; % adaptive controller gain term (rate of convergence)
constants.c = 1; % input the bound on C here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIRED/INITIAL CONDITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define the initial state of rigid body
constants.x0 = [10;10;0];
constants.v0 = zeros(3,1);

% desired position state
constants.xd = zeros(3,1);
constants.vd = zeros(3,1);

% R0 = ROT1(0*pi/180)*ROT2(45*pi/180)*ROT3(180*pi/180);
constants.q0 = [-0.188 -0.735 -0.450 -0.471];
% constants.qd = [-0.59 0.67 0.21 -0.38]; % from lee/meshbahi paper
constants.qd = [0 0 0 1];

% constants.R0 = quat2dcm(constants.q0)';
% constants.Rd = quat2dcm(constants.qd)';

% constants.R0 = ROT1(0*pi/180)*ROT3(0*pi/180); % avoid single constraint
% constants.Rd = ROT3(90*pi/180);

constants.R0 = ROT1(0*pi/180)*ROT3(225*pi/180); % avoid multiple constraints
constants.Rd = eye(3,3);


R0 = constants.R0;
w0 = zeros(3,1);
delta_est0 = zeros(3,1); 
initial_state = [constants.x0;constants.v0;constants.R0(:);w0; delta_est0];

% simulation timespan
tspan = linspace(0,30,100);