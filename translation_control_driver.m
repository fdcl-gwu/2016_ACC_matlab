% 3 November 2015
% simulate translational obstacle avoidance

clear all
clc
close all

% define initial condition
x0 = [10;10];
% desired state
constants.xd = zeros(2,1);
% define error constants
% obstacle
constants.xo = [5 ;...
      5.1 ]; % obstacles are the column vectors
num_obs = size(constants.xo,2);

constants.m_sc = 1;
%% error function parameters
constants.alpha = 10;
constants.beta = 0.5;
constants.N = eye(2,2);
constants.P = eye(2,2);

wn = 4;
zeta = 0.7;

constants.kx = constants.m_sc*2*zeta*wn;
constants.kv = constants.m_sc*wn^2;

% simulate system
initial_state = [x0;zeros(2,1)];
tspan = linspace(0,30,100);
[t, state] = ode45(@(t,state)trans_dynamics(t,state,constants),tspan, initial_state);
% plot outputs
pos = state(:,1:2);
vel = state(:,3:4);

figure
hold all
grid on
title('Position','interpreter','latex')
xlabel('x','interpreter','latex')
ylabel('y','interpreter','latex')
plot(pos(:,1),pos(:,2))

figure
subplot(2,1,1)
hold all
grid on
title('X','interpreter','latex')
xlabel('t','interpreter','latex')
ylabel('x','interpreter','latex')
plot(t,pos(:,1))
subplot(2,1,2)
hold all
grid on
title('Y','interpreter','latex')
xlabel('t','interpreter','latex')
ylabel('y','interpreter','latex')
plot(t,pos(:,2))