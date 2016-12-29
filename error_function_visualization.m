% 28 Dec 16 
% Visulalize error function behavior to try and find some relationship for
% lypanov stability proof
close all
clc
clearvars

addpath(genpath('./utilities'));

fontsize = 18;
fontname = 'Times';
figx = 680;
figy = 224;
figw = 800;
figh = 600;

fig_size = [figx,figy,figw,figh];

con_angle = 10*pi/180; % angular constraint
x = [1;0;1]; % axis of rotation
x = x/norm(x);
alpha = 20;

sen = [1;0;0]; % body fixed frame
con = [-1;0;0]; % inertial frame
con = con/norm(con);

R_des = eye(3,3);
G = diag([0.9 1 1.1]);

% functions for config error terms
C = [1,2,3;
    2,3,1;
    3,1,2]; % indices for rodriguez formula


% create surface plot of \Psi = A B and e_{R}
density = 50;
angle2con = linspace(pi,con_angle,density); % dot product - ksi
angle2des = linspace(0,2*pi,density); % angle to desired attitude - phi
[phi, xi] = meshgrid(angle2des, angle2con);

eRA_array = zeros(size(phi));
eRB_array = zeros(size(phi));
B_array = zeros(size(phi));
A_array = zeros(size(phi));
psi_array = zeros(size(phi));

for ii = 1:length(angle2con)% loop over angle between R^T v and r
    for jj = 1:length(angle2des)% loop over angle between R and R_D (using Rodriguez formula)
        % store height into arrays
        eRA_array(ii,jj) = eRA(phi(ii,jj),G,x,C);
        eRB_array(ii,jj) = eRB(alpha,xi(ii,jj),con_angle);
        A_array(ii,jj) = A(phi(ii,jj),G,x,C);
        B_array(ii,jj) = B(alpha,xi(ii,jj),con_angle);

    end
end

% create surface
figure('Position',fig_size)
hold all
grid on
title('$A(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$A(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,A_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('$B(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$B(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,B_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('$e_{R_A}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$e_{R_A}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,eRA_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('$e_{R_B}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$e_{R_B}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,eRB_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('$\Psi = A(R) B(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$\Psi$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,A_array.*B_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('$e_R = A(R) e_{R_B} + B(R) e_{R_A}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$\phi$ Desired','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\xi$ Constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$e_R$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(phi.*180/pi,xi.*180/pi,A_array.*eRB_array+B_array.*eRA_array)
axis([0 360 0 180 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

function A_out = A(phi,G,x,C)

    % loop over the set C
    summation = 0;
    for hh = 1:3
        ii = C(hh,1);
        jj = C(hh,2);
        kk = C(hh,3);
        
        summation = summation + (G(ii,ii)+G(jj,jj))*x(kk)^2;
    end
    
    A_out = (1-cos(phi))/(2*phi^2)*summation;
end

function eRA_out = eRA(phi,G,x,C)
    % loop over the set C
    summation_1 = 0;
    summation_2 = 0;
    
    for hh = 1:3
        ii = C(hh,1);
        jj = C(hh,2);
        kk = C(hh,3);
        
        summation_1 = summation_1 + (G(ii,ii)-G(jj,jj))^2*x(ii)^2*x(jj)^2;
        summation_2 = summation_2 + (G(ii,ii)+G(jj,jj))^2*x(kk)^2;
    end
    
    eRA_out = (1-cos(phi))^2/(4*phi^4)*summation_1 + (sin(phi))^2/(4*phi^2)*summation_2;
end

function B_out = B(alpha,ksi,theta)
    B_out = 1 - 1/alpha * log((cos(theta)-ksi)/(1+cos(theta)));
end

function eRB_out = eRB(alpha,ksi,theta)
    eRB_out = 1/alpha*1/abs(ksi-cos(theta))*sin(ksi)
end