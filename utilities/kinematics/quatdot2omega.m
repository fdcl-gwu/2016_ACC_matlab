
function w = quatdot2omega(q,qd)

% form the E matrix
E = [q(4) -q(3) q(2) q(1);...
    q(3) q(4) -q(1) q(2);...
    -q(2) q(1) q(4) q(3);...
    -q(1) -q(2) -q(3) q(4)];

w = 2*qd*E;

w = w(1:3);