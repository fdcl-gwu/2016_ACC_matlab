% log barrier function
close all
clc
clear all

addpath(genpath('./utilities'));

fontsize = 18;
fontname = 'Times';
figx = 680;
figy = 224;
figw = 800;
figh = 600;

fig_size = [figx,figy,figw,figh];

con_angle = cos(10*pi/180); % angular constraint
angle = -1:1e-2:con_angle; % dot product
x = 0:0.001:1;
alpha = 20;

sen = [1;0;0]; % body fixed frame
con = [1;1;0]; % inertial frame
con = con/norm(con);

R_des = eye(3,3);
G = diag([0.9 1 1.1]);
% cylindrical projection of attitude ROT3(lon)*ROT2(lat)
den = 50;
lon = linspace(-180,180,den)*pi/180;
lat = linspace(-90, 90, den)*pi/180;

[X, Y] = meshgrid(lon, lat);

sen_array = zeros(3,den^2);
er_attract_array = zeros(3,den^2);
psi_avoid_array = zeros(size(X));
psi_attract_array = zeros(size(X));
psi_total_array = zeros(size(X));
for ii = 1:length(lon)
    for jj = 1:length(lat)
        % rotate body vector to inertial frame
        R_b2i = ROT2(lat(jj))'*ROT3(lon(ii))';
        
        % save the vector into a big array
        sen_inertial = R_b2i * sen;
        sen_array(:,(ii-1)*den + jj) = sen_inertial;
        
        psi_attract = 1/2*trace(G*(eye(3,3)-R_des'*R_b2i));
        
        if dot(sen_inertial,con) < con_angle
            % calculate error function
            psi_avoid = -1/alpha*log(-((dot(sen_inertial,con)-con_angle)/(1+con_angle))) + 1;
        else
            psi_avoid = 10;
            
        end
        psi_total = psi_attract*psi_avoid;
        
        er = 1/2*vee_map(G*R_des'*R_b2i - R_b2i'*R_des*G);
        
        er_attract_array(:,(ii-1)*den + jj) = er';
        psi_avoid_array(ii,jj) = (psi_avoid);
        psi_attract_array(ii,jj) = psi_attract;
        psi_total_array(ii,jj) = psi_total;
        
    end
end
% pick a desired attitude (I)
% calculate \Psi ( R, R_d)
% plot \Psi with the addition of the log barrier function



g = 1+-1/alpha*log(-(angle-con_angle)/(1+con_angle));
eRB = abs(1/alpha*sin(acos(angle))./(angle-con_angle)); % norm of eRB

figure('Position',fig_size)
plot(acos(angle)*180/pi, real(g))
xlabel('Angle to constraint','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('Barrier','interpreter','latex','FontName',fontname,'FontSize',fontsize)
title('Logarithmic Barrier Function','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on
hold all
plot(acos(angle)*180/pi,eRB)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('Sensor Inertial Pointing Direction','interpreter','latex','FontName',fontname,'FontSize',fontsize)
plot3(sen_array(1,:),sen_array(2,:), sen_array(3,:),'b.')
% quiver3(sen_array(1,:),sen_array(2,:),sen_array(3,:),er_attract_array(1,:),er_attract_array(2,:),er_attract_array(3,:),2);
line([0 con(1)],[0 con(2)],[0 con(3)],'color','red','linewidth',10);
xlabel('X','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('Y','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('Z','interpreter','latex','FontName',fontname,'FontSize',fontsize)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('Avoid','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('Longitude ($\lambda$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('Latitude ($\beta$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$B(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(X.*180/pi,Y.*180/pi,(psi_avoid_array))
axis([-180 180 -90 90 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('Attract','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('Longitude ($\lambda$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('Latitude ($\beta$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$A(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(X.*180/pi,Y.*180/pi,psi_attract_array)
axis([-180 180 -90 90 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);

figure('Position',fig_size)
hold all
grid on
title('Total','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('Longitude ($\lambda$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('Latitude ($\beta$)','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('$\Psi(R)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
surf(X.*180/pi,Y.*180/pi,psi_total_array)
axis([-180 180 -90 90 0 3])
view(3)
set(gca,'FontName',fontname,'FontSize',fontsize);