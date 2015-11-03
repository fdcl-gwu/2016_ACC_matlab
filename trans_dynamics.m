function [state_dot] = trans_dynamics(t,state,constants)
% extract out the state
pos = state(1:2);
vel = state(3:4);

% constants
m_sc = constants.m_sc;
xd = constants.xd;
xo = constants.xo;
veld = zeros(2,1);

P = constants.P;
N = constants.N;
alpha = constants.alpha;
beta = constants.beta;

kx = constants.kx;
kv = constants.kv;

f_ext = zeros(3,1);

% compute the control
dA = P*(pos-xd);
dR = -alpha/beta*exp(-(pos-xo)'*N*(pos-xo)/beta)*N*(pos-xo);

A = 1/2*(pos-xd)'*P*(pos-xd);
R = 1+alpha/2*exp(-(pos-xo)'*N*(pos-xo)/beta);

err_pos = dA*R + dR*A;
err_vel = vel - veld;

u = -kx*err_pos - kv*err_vel;
% derivatives
pos_dot = vel;
vel_dot = u/m_sc;

state_dot = [pos_dot;vel_dot];
end