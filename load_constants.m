% 22 September 2015
% load constants for simulation

constants.scenario = 'single'; % or 'single'
constants.avoid_switch = 'true';
constants.dist_switch = 'true';
constants.adaptive_switch = 'true';

% ACC/IJCAS Simulation for Fig 2 is
% constants.scenario = 'multiple'; % or 'single'
% constants.avoid_switch = 'true';
% constants.dist_switch = 'true';
% constants.adaptive_switch = 'false';


% constants for plotting/animations
constants.animation_type = 'none'; % or 'movie' or 'none'
constants.filename = 'multiple_avoid';

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
% controller parameters
constants.G = diag([0.9 1 1.1]);

% con = -1+2.*rand(3,constants.num_con); % inertial frame vectors (3XN)
% from [1] U. Lee and M. Mesbahi. Spacecraft Reorientation in Presence of Attitude Constraints via Logarithmic Barrier Potentials. In 2011 AMERICAN CONTROL CONFERENCE, Proceedings of the American Control Conference, pages 450?455, 345 E 47TH ST, NEW YORK, NY 10017 USA, 2011. Boeing; Bosch; Corning; Eaton; GE Global Res; Honeywell; Lockheed Martin; MathWorks; Natl Instruments; NT-MDT; United Technol, IEEE. American Control Conference (ACC), San Fransisco, CA, JUN 29-JUL 01, 2011.
% con = [0.174    0   -0.853 -0.122;...
%     -0.934   0.7071    0.436 -0.140;...
%     -0.034   0.7071   -0.286 -0.983];
% column vectors to define constraints
% zeta = 0.7;
% wn = 0.2;
% constants.kp = wn^2;
% constants.zeta = 2*zeta*wn;
% constants.kp = 0.0424; % wn^2
% constants.kp = 0.4;
% constants.kv = 0.296; % 2*zeta*wn 
constants.kp = 0.4;
constants.kv = 0.296; % 2*zeta*wn 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONSTRAINT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

constants.sen = [1;0;0]; % body fixed frame
% define a number of constraints to avoids

switch constants.scenario
    case 'multiple'
        con = [0.174    0.4   -0.853 -0.122;...
            -0.934   0.7071    0.436 -0.140;...
            -0.034   0.7071   -0.286 -0.983];
        constants.con_angle = [40;40;40;20]*pi/180;
    case 'single'
        con = [1/sqrt(2);1/sqrt(2);0];
        constants.con_angle = 12*pi/180;
end
constants.con = con./repmat(sqrt(sum(con.^2,1)),3,1); % normalize

constants.alpha = 15; % use the same alpha for each one
constants.num_con = size(constants.con,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ADAPTIVE CONTROL FOR DISTURBANCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disturbance terms

constants.W = eye(3,3);
constants.delta = @(t) 0.2 + 0.02*[sin(9*t);cos(9*t);1/2*(sin(9*t)+cos(9*t))];
constants.kd = 0.5; % adaptive controller gain term (rate of convergence)
constants.c = 1; % input the bound on C here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIRED/INITIAL CONDITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define the initial state of rigid body

% R0 = ROT1(0*pi/180)*ROT2(45*pi/180)*ROT3(180*pi/180);
constants.q0 = [-0.188 -0.735 -0.450 -0.471];
% constants.qd = [-0.59 0.67 0.21 -0.38]; % from lee/meshbahi paper
constants.qd = [0 0 0 1];

% constants.R0 = quat2dcm(constants.q0)';
% constants.Rd = quat2dcm(constants.qd)';

switch constants.scenario
    case 'multiple'
        constants.R0 = ROT1(0*pi/180)*ROT3(225*pi/180); % avoid multiple constraints
        constants.Rd = eye(3,3);
    case 'single'
        constants.R0 = ROT1(0*pi/180)*ROT3(0*pi/180); % avoid single constraint
        constants.Rd = ROT3(90*pi/180);
end





R0 = constants.R0;
w0 = zeros(3,1);
delta_est0 = zeros(3,1); 
initial_state = [constants.R0(:);w0; delta_est0];

% simulation timespan
tspan = linspace(0,20,1000);