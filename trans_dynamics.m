function [state_dot] = trans_dynamics(t,state,constants)
% extract out the state
pos = state(1:3);
vel = state(4:6);

% constants
m_sc = constants.m_sc;
xd = constants.xd;
xo = constants.xo;
veld = zeros(3,1);

det_shell = 1; % detection shell radius

P = constants.P;
N = constants.N;
alpha = constants.alpha;
beta = constants.beta;

kx = constants.kx;
kv = constants.kv;

f_ext = zeros(3,1);

%% compute the control
dA = P*(pos-xd);
dR = -alpha*exp(-(pos-xo)'*inv(N)*(pos-xo))*inv(N)*(pos-xo);

A = 1/2*(pos-xd)'*P*(pos-xd);
R = 1+alpha/2*exp(-(pos-xo)'*inv(N)*(pos-xo));

err_pos = dA*R + dR*A;
err_vel = vel - veld;
% u = -kx*err_pos - kv*err_vel;
%% Gyroscopic force to avoid obstacle
% compute distance to obstacle
dist_obs = xo - pos;

% check if we are within the detection shell
S = zeros(3,3);
if norm(dist_obs) < (det_shell + beta)
    % decide on direction to rotate
    vec = hat_map(dist_obs)*vel;

    if norm(vec) > 1e-6
        S = hat_map(vec);
    else
        S = hat_map([1 1 1]);
    end

end

u = -kx*err_pos - kv*err_vel + 1/2*norm(constants.initial_state(1:3)-xd)^2*S*vel
%% state update derivatives
pos_dot = vel;
vel_dot = u/m_sc;

state_dot = [pos_dot;vel_dot];
end