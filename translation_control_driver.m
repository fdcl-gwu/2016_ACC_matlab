% 3 November 2015
% simulate translational obstacle avoidance

clc
close all

restoredefaultpath
addpath(genpath('./utilities'))
% define initial condition
x0 = [10;10;0];
% desired state
constants.xd = zeros(3,1);
% define error constants
% obstacle
constants.xo = [5 ;...
      5;0 ]; % obstacles are the column vectors
num_obs = size(constants.xo,2);

constants.m_sc = 1;
%% error function parameters
constants.alpha = 5; % scale factor
constants.beta = 1; % std dev
constants.N = diag(constants.beta*ones(1,3));
constants.P = eye(3,3);

wn = 4;
zeta = 0.7;

constants.kx = constants.m_sc*2*zeta*wn;
constants.kv = constants.m_sc*wn^2;

% simulate system
initial_state = [x0;0;0;0];
constants.initial_state = initial_state;

tspan = linspace(0,30,100);
[t, state] = ode45(@(t,state)trans_dynamics(t,state,constants),tspan, initial_state);
% plot outputs
pos = state(:,1:3);
vel = state(:,4:6);

figure
hold all
grid on
title('Position','interpreter','latex')
xlabel('x','interpreter','latex')
ylabel('y','interpreter','latex')
zlabel('z','interpreter','latex')
plot3(pos(:,1),pos(:,2),pos(:,3))

% plot the obstacle as a ellipsoid
plot_gaussian_ellipsoid(constants.xo, constants.N);

figure
subplot(3,1,1)
hold all
grid on
title('X','interpreter','latex')
xlabel('t','interpreter','latex')
ylabel('x','interpreter','latex')
plot(t,pos(:,1))

subplot(3,1,2)
hold all
grid on
title('Y','interpreter','latex')
xlabel('t','interpreter','latex')
ylabel('y','interpreter','latex')
plot(t,pos(:,2))

subplot(3,1,3)
hold all
grid on
title('Z','interpreter','latex')
xlabel('t','interpreter','latex')
ylabel('z','interpreter','latex')
plot(t,pos(:,3))