% 4 September 2015
% This is using $ \bar{e}_\Omega $ and is more complicated and probalby
% incorrect

close all
clc

% define bounds on the initial condition (limit the maximum of \Psi)
theta_int = 90*pi/180; % initial deviation from desired attitude no more than 45 deg

load_constants

kr = constants.kp;
kw = constants.kv;
kd = constants.kd;
hm = max(eig(constants.J));
c = constants.c;
r = constants.sen;
v = constants.con;
G = constants.G;
theta = constants.con_angle;
alpha = constants.alpha;
W = sqrt(trace(constants.W'*constants.W));

% calculate how this bound maps to \Psi
R0 = ROT3(theta_int);
w0 = zeros(3,1);
theta_est = zeros(3,1);
[~, ~, ~, ~, ~, Psi_max, ~, ~] ...
    = controller(0,[R0(:);w0;theta_est], constants);

% figure out bound on $ r^T R^T v \leq \beta$
beta_max = r'*R0'*v;

% bounds on attractive error function from previous Dr. Lee paper
h1 = min([G(1,1)+G(2,2),G(2,2)+G(3,3),G(3,3)+G(1,1)]);
h2 = max([(G(1,1)-G(2,2))^2,(G(2,2)-G(3,3))^2,(G(3,3)-G(1,1))^2]);
h3 = max([(G(1,1)+G(2,2))^2,(G(2,2)+G(3,3))^2,(G(3,3)+G(1,1))^2]);
h4 = max([G(1,1)+G(2,2),G(2,2)+G(3,3),G(3,3)+G(1,1)]);
h5 = min([(G(1,1)+G(2,2))^2,(G(2,2)+G(3,3))^2,(G(3,3)+G(1,1))^2]);
b1 = h1/(h2+h3);


% calculate stability terms to see if my Lyapunov function is correct


% calculate H bound
H = Psi_max/sqrt(2) * trace(G) + 2 * sqrt(Psi_max/b1) * (sin( theta))/(alpha*(beta_max - cos(theta))) ...
    + Psi_max*(((beta_max^2+1)*(beta_max-cos(theta))^2 + 1 + beta_max^2*(beta_max^2-2))/(alpha^2*(beta_max-cos(theta))^4));

% M matrix is composed of constants
M=zeros(2,2);
M(1,1) = c*kr;
M(1,2) = kw*c/2;
M(2,1) = M(1,2);
M(2,2) = kw-c*hm*H;

fprintf('c must be less than %6.4f\n',(4*kr*kw)/(kw^2 + 4*kr*hm*H))
% M = zeros(3,3);
% M(1,1) = c * kr - hm*kd/4/kw *(c^2 + kr/kw*c)*W;
% M(1,2) = -hm*kd/8/kw*W*(2*c+kr/kw)+kw*c/2;
% M(1,3) = -hm/16/kw^2*kd*c*W^2;
% M(2,2) = kw - hm*kd/4/kw*W -c*hm*H;
% M(2,3) = -hm/16/kw^2*kd*W^2-c*hm*H/4/kw*W;
% M(3,3) = 1/4/kw*W^2;
% 
% M(2,1) = M(1,2);
% M(3,1) = M(1,3);
% M(3,2) = M(2,3);

% % test the 3 conditions from teh leading principle minors
% % first condition 
% % - (W*hm*kd*c^2)/(4*kw) + ((4*kr*kw^2 - W*hm*kd*kr)*c)/(4*kw^2)
% a2 = - (W*hm*kd)/(4*kw);
% a1 = ((4*kr*kw^2 - W*hm*kd*kr))/(4*kw^2);
% p = [a2 a1 0];
% cond1_roots = roots(p);
% 
% x = linspace(-5+min(cond1_roots),cond1_roots(end)+5,100);
% 
% figure
% plot(x,polyval(p,x))
% title('Condition 1')
% xlabel('$ c $')
% ylabel('First Cond')
% 
% % second condition
% % (H*W*hm^2*kd*c^3)/(4*kw) - ((- 16*H*W*kd*kr*hm^2*kw^2 + 64*H*kr*hm*kw^4 + 16*kw^6)*c^2)/(64*kw^4) + ((64*kr*kw^5 - 24*W*hm*kd*kr*kw^3)*c)/(64*kw^4) - (W^2*hm^2*kd^2*kr^2)/(64*kw^4)
% a3 = (H*W*hm^2*kd)/(4*kw);
% a2 = -((- 16*H*W*kd*kr*hm^2*kw^2 + 64*H*kr*hm*kw^4 + 16*kw^6))/(64*kw^4);
% a1 = ((64*kr*kw^5 - 24*W*hm*kd*kr*kw^3))/(64*kw^4);
% a0 = - (W^2*hm^2*kd^2*kr^2)/(64*kw^4);
% p2 = [a3 a2 a1 a0];
% cond2_roots = roots(p2);
% 
% x = linspace(-5+min(cond2_roots),max(cond2_roots)+5,100);
% figure
% plot(x,polyval(p2,x))
% title('Condition 2 ')
% xlabel('$ c $')
% ylabel('Second cond')
% 
% % third condition
% % (H^2*W^3*hm^3*kd*c^4)/(64*kw^3) + (W^2*(4*kr*H^2*W*hm^3*kd*kw - 16*kr*H^2*hm^2*kw^3 + H*W^2*hm^3*kd^2*kw + 20*H*W*hm^2*kd*kw^3)*c^3)/(256*kw^5) - (W^2*(- H*kr*W^2*hm^3*kd^2 - 8*H*kr*W*hm^2*kd*kw^2 + 64*H*kr*hm*kw^4 + 16*kw^6)*c^2)/(256*kw^5) - (W^2*(kr*W^2*hm^2*kd^2*kw + 24*kr*W*hm*kd*kw^3 - 64*kr*kw^5)*c)/(256*kw^5) - (W^4*hm^2*kd^2*kr^2)/(256*kw^5)
% a4 = (H^2*W^3*hm^3*kd)/(64*kw^3);
% a3 = (W^2*(4*kr*H^2*W*hm^3*kd*kw - 16*kr*H^2*hm^2*kw^3 + H*W^2*hm^3*kd^2*kw + 20*H*W*hm^2*kd*kw^3))/(256*kw^5);
% a2 = - (W^2*(- H*kr*W^2*hm^3*kd^2 - 8*H*kr*W*hm^2*kd*kw^2 + 64*H*kr*hm*kw^4 + 16*kw^6))/(256*kw^5);
% a1 = - (W^2*(kr*W^2*hm^2*kd^2*kw + 24*kr*W*hm*kd*kw^3 - 64*kr*kw^5))/(256*kw^5);
% a0 = - (W^4*hm^2*kd^2*kr^2)/(256*kw^5);
% 
% p3 = [a4 a3 a2 a1 a0];
% cond3_roots = roots(p3);
% 
% x = linspace(-5+min(cond3_roots),max(cond3_roots)+5,500);
% figure
% plot(x,polyval(p3,x))
% title('Condition 3 ')
% xlabel('$ c $')
% ylabel('Third cond')


