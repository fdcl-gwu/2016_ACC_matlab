% 31 August 2015
% driver for constrained attitude control 

% clear all
clc
% close all

% add path to utilities
restoredefaultpath
addpath(genpath('./utilities'));

% define constants/properties of rigid body
constants.m_sc = 100;
m_sc = constants.m_sc;

constants.J = [1.059e-2 -5.156e-6 2.361e-5;...
                -5.156e-6 1.059e-2 -1.026e-5;
                2.361e-5 -1.026e-5 1.005e-2];
% % % from Farhad ASME paper
% constants.J = [ 5.5711 0.0618 -0.0251; ...
%                 0.06177 5.5757 0.0101;...
%                 -0.02502 0.01007 1.05053] * 1e-2;

% constants.J = diag([694 572 360]);

J = constants.J;

% controller parameters
constants.G = diag([0.9 1 1.1]);
constants.sen = [1;0;0]; % body fixed frame
% define a number of constraints to avoid

% con = -1+2.*rand(3,constants.num_con); % inertial frame vectors (3XN)
% con = [0.174    0   -0.853 -0.122;...
%     -0.934   0.7071    0.436 -0.140;...
%     -0.034   0.7071   -0.286 -0.983];
% con = [0.174    0   -0.853 -0.122;...
%     -0.934   0.7071 -0.236 -0.140;...
%     -0.034   0.7071 -0.286 -0.983];
con = [0 0;...
      1 -1;...
       0 0];
constants.con_angle = [10;10]*pi/180;
constants.con = con./repmat(sqrt(sum(con.^2,1)),3,1); % normalize
constants.alpha = 20; % use the same alpha for each one
constants.num_con = size(constants.con,2);
% zeta = 0.7;
% wn = 0.2;
% constants.kp = wn^2;
% constants.zeta = 2*zeta*wn;
% constants.kp = 0.0424; % wn^2
% constants.kp = 0.4;
% constants.kv = 0.296; % 2*zeta*wn 
constants.kp = 0.4;
constants.kv = 0.296; % 2*zeta*wn 
% constants.kp = 0.7;
% constants.kv = 0.12;

% disturbance terms
constants.W = eye(3,3);
% constants.theta = zeros(3,1);
constants.theta = [0.1;0.1;0.1];
constants.gam = 4; % adaptive controller gain term (rate of convergence)

% define the initial state of rigid body
% pick a random euler angle sequence that satisfies the constraints
% R0 = ROT1(0*pi/180)*ROT2(45*pi/180)*ROT3(180*pi/180);
constants.q0 = [-0.188 -0.735 -0.450 -0.471];
% constants.qd = [-0.59 0.67 0.21 -0.38]; % from lee/meshbahi paper
constants.qd = [0 0 0 1];

R0 = ROT2(20*pi/180)*ROT3(175*pi/180);
w0 = zeros(3,1); % initial angular velocity
theta_est0 = zeros(3,1); 
initial_state = [R0(:);w0; theta_est0];

constants.R0 = R0;
constants.Rd = eye(3,3);
% simulation timespan
tspan = linspace(0,20,1e3);

% propogate a chief and deputy spacecraft (continuous time system)
[t, state] = ode45(@(t,state)dynamics(t,state,constants),tspan, initial_state);
% calculate the relative position and attitude of deputy wrt chief

% extract out the states

% loop to save the Body to Inertial rotation matrix into a big array
% calculate the desired trajectory
R_b2i = zeros(3,3,length(tspan));
u_f = zeros(3,length(tspan));
u_m = zeros(3,length(tspan));
R_des = zeros(3,3,length(tspan));
ang_vel_des = zeros(3,length(tspan));
ang_vel_dot_des = zeros(3,length(tspan));
Psi = zeros(length(tspan),1);
err_att = zeros(3,length(tspan));
err_vel = zeros(3,length(tspan));

ang_vel = state(:,10:12);
theta_est = state(:,13:15);
for ii = 1:length(tspan)
   R_b2i(:,:,ii) = reshape(state(ii,1:9),3,3); 

   [u_f(:,ii), u_m(:,ii), R_des(:,:,ii), ang_vel_des(:,ii), ang_vel_dot_des(:,ii), Psi(ii), err_att(:,ii), err_vel(:,ii)] ...
    = controller(t(ii),state(ii,:)', constants);
    
  
end


plot_outputs

