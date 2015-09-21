clc

% v = [1;0;0];
% v = v/norm(v);
% r = [1/2;sqrt(3)/2;0];
% r = r/norm(r);
% R = ROT2(0*pi/180);
% 
% theta = 60*pi/180;
% 
% a = v'*R*r*eye(3,3);
% b = R'*v*r';
% c = R'*hat_map(v)*R*r*v'*R*hat_map(r);
% 
% ata = a'*a;
% 
% atc = trace(c)*v'*R*r/(r'*R'*v - cos(theta))
% atb = a'*b;
% bta = b'*a;
% btc = trace(b'*c);
% btb = b'*b;
% 
% ctc =  c'*c ;
% trace_ctc = (1 - 2*(v'*R*r)^2 + (v'*R*r)^4)/(r'*R'*v - cos(theta))^2;

% lyapunov derivative

syms kw hm kd c kr H W
syms ew ed er

z = transpose([er ew ed ]);

M(1,1) = c * kr - hm*kd/4/kw *(c^2 + kr/kw*c)*W;
M(1,2) = -hm*kd/8/kw*W*(2*c+kr/kw)+kw*c/2;
M(1,3) = -hm/16/kw^2*kd*c*W^2;
M(2,2) = kw - hm*kd/4/kw*W -c*hm*H;
M(2,3) = -hm/16/kw^2*kd*W^2-c*hm*H/4/kw*W;
M(3,3) = 1/4/kw*W^2;

M(2,1) = M(1,2);
M(3,1) = M(1,3);
M(3,2) = M(2,3);

 Vdot_quad = transpose(z)*M*z;

 
  