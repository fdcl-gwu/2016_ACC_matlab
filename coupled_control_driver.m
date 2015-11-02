% 31 August 2015
% driver for constrained attitude control 

clear all
clc
close all

% add path to utilities
restoredefaultpath
addpath(genpath('./utilities'));

load_constants

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
delta_est = state(:,13:15);
for ii = 1:length(tspan)
   R_b2i(:,:,ii) = reshape(state(ii,1:9),3,3); 

   [u_f(:,ii), u_m(:,ii), R_des(:,:,ii), ang_vel_des(:,ii), ang_vel_dot_des(:,ii), Psi(ii), err_att(:,ii), err_vel(:,ii)] ...
    = controller(t(ii),state(ii,:)', constants);
end


plot_outputs

% draw_cad