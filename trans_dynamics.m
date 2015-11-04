function [state_dot] = trans_dynamics(t,state,constants)
% extract out the state
pos = state(1:3);
vel = state(4:6);

% constants
m_sc = constants.m_sc;
xd = constants.xd;
xo = constants.xo;
veld = zeros(3,1);

P = constants.P;
N = constants.N;
alpha = constants.alpha;
beta = constants.beta;

kx = constants.kx;
kv = constants.kv;

f_ext = zeros(3,1);

% compute the control
dA = P*(pos-xd);
dR = -alpha*exp(-(pos-xo)'*inv(N)*(pos-xo))*inv(N)*(pos-xo);

A = 1/2*(pos-xd)'*P*(pos-xd);
R = 1+alpha/2*exp(-(pos-xo)'*inv(N)*(pos-xo));

err_pos = dA*R + dR*A;
err_vel = vel - veld;

u = -kx*err_pos - kv*err_vel;
% derivatives
pos_dot = vel;
vel_dot = u/m_sc;

state_dot = [pos_dot;vel_dot];
end