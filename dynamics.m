% 31 August 2015
% ODE function for constrained attitude stabilization

function [state_dot] = dynamics(t, state, constants)

% constants
m_sc = constants.m_sc;
J = constants.J;
kd = constants.kd;
W = constants.W;

% redefine the state vector
R = reshape(state(1:9),3,3); % rotation matrix from body to inertial frame
ang_vel = state(10:12);
delta_est = state(13:15); % adaptive control term to estimate fixed disturbance

% calculate external force and moment
[~, m] = ext_force_moment(t,state,constants);

[~, u_m, ~, ~, ~, ~, err_att, err_vel] ...
    = controller(t,state,constants);

% differential equations of motion

R_dot = R*hat_map(ang_vel);
ang_vel_dot =J \ ( m + u_m - cross(ang_vel,J*ang_vel));
% theta_est_dot =  gam/2 * W' *(err_vel+ constants.kp/constants.kv * err_att);
% theta_est_dot =  gam/2 * W' *(err_vel+ (constants.c + constants.kp/constants.kv)* err_att);
theta_est_dot = kd * W' *(err_vel + constants.c*err_att);
% output the state derivative
state_dot = [R_dot(:);ang_vel_dot; theta_est_dot];

end

function [f, m] = ext_force_moment(t,state, constants)

% redefine the state vector
R = reshape(state(1:9),3,3); % rotation matrix from body to inertial frame
ang_vel = state(10:12);

% constants
m_sc = constants.m_sc;
J = constants.J;
W = constants.W;
delta = constants.delta;

% calculate external moment and force
f = zeros(3,1);

% add a constant disturbance
% m = 3*mu/norm(pos)^3 * cross(R_body2lvlh'*a1_hat,J*R_body2lvlh'*a1_hat);
switch constants.dist_switch
    case 'true'
        m = zeros(3,1) + W*delta(t);
    case 'false'
        m = zeros(3,1);
end
end

