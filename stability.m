% 4 September 2015

% calculate stability terms to see if my Lyapunov function is correct

kr = constants.kp;
kw = constants.kv;
kd = constants.gam;
hm = max(eig(constants.J));

% M matrix is composed of constants
M = zeros(3,3);
M(1,2) = -(kr*kd*hm)/(8 * kw^2);
M(2,1) = M(1,2);
M(2,2) = kw - (kd*hm)/(4*kw);
M(2,3) = -(kd*hm)/(16*kw^2);
M(3,2) = M(2,3);
M(3,3) = 1/(4*kw);