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
angle2con = pi:1e-2:con_angle; % dot product - ksi
angle2des = -pi:1e-2:pi; % angle to desired attitude - phi
x = [0;1;1]; % axis of rotation
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

% loop over angle between R^T v and r

% loop over angle between R and R_D (using Rodriguez formula)

% store height into arrays

% create surface

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