% 2 November 2015 - translation obstacle for fully actuated rigid body

% (2D case for visualization)
close all
clc
clear all
% initial position

xd = [0;0];

% obstacle
xo = [5 -5;...
      5 -5]; % obstacles are the column vectors
num_obs = size(xo,2);

%% error function parameters
alpha = 10;
beta = 0.5;
N = eye(2,2);
P = eye(2,2);

%% grid of initial conditions
xrange = linspace(-10,10,100);
yrange = linspace(-10,10,100);
[X, Y] = meshgrid(xrange,yrange);

attract = zeros(size(X));
repulse = zeros(size(X));
psi = zeros(size(X));
for ii = 1:length(xrange)
    for jj = 1:length(yrange)
        x0 = [xrange(ii);yrange(jj)];
        % calculate the error functions at this location
        attract_pot = 1/2*(x0-xd)'*P*(x0-xd);
        repulse_pot = 1;
        for kk = 1:num_obs
            repulse_pot = repulse_pot+alpha/2*exp(-(x0-xo(:,kk))'*N*(x0-xo(:,kk))/beta);
        end
        config_error = attract_pot*repulse_pot;
        
        attract(ii,jj) = attract_pot;
        repulse(ii,jj) = repulse_pot;
        psi(ii,jj) = config_error;
    end
end

% plot the results
figure
hold all
grid on
title('Avoid','interpreter','latex')
xlabel('X ','interpreter','latex')
ylabel('Y','interpreter','latex')
surf(X,Y,repulse)

figure
hold all
grid on
title('Attract','interpreter','latex')
xlabel('X','interpreter','latex')
ylabel('Y','interpreter','latex')
surf(X,Y,attract)

figure
hold all
grid on
title('Total','interpreter','latex')
xlabel('X','interpreter','latex')
ylabel('Y','interpreter','latex')
surf(X,Y,psi)
