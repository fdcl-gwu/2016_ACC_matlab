% 11 June 15
% plot data
close all
num_figs = 10;
fig_handle = zeros(num_figs,1);
for ii = 1:num_figs
    fig_handle(ii) = figure;
end

fontsize = 18;
fontname = 'Times';

fig_index = 0;
% plot the position
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

subplot(3,1,1)
title('Position','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$x $','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,pos(:,1));

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% ylabel('$y$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,pos(:,2));

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$z $','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,pos(:,3));

% plot the velocity
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

subplot(3,1,1)
title('Velocity','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\dot{x}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,vel(:,1));

subplot(3,1,2)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\dot{y}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,vel(:,2));

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\dot{z}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,vel(:,3));

% plot the attitude error vector
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

subplot(3,1,1)
% title('Attitude error vector','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{R_1}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_att(1,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,2)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{R_2}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_att(2,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{R_3}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_att(3,:));

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot the attitude error \Psi
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

% title('$\Psi$ error ','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\Psi$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,Psi);

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot the angular velocity error
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

subplot(3,1,1)
% title('Angular velocity error vector','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{\Omega_1}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_vel(1,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,2)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{\Omega_2}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_vel(2,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$e_{\Omega_3}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,err_vel(3,:));

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot the control input
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

subplot(3,1,1)
% title('Control Input','interpreter','latex','FontName',fontname,'FontSize',fontsize)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$u_{1}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,u_m(1,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,2)
% xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$u_{2}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,u_m(2,:));
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$u_{3}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,u_m(3,:));

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot the desired adn actual angular velocities
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector


subplot(3,1,1)
title('Angular Velocity','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\Omega_{1}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,ang_vel(:,1),'b');
plot(t,ang_vel_des(1,:),'r');
l=legend('Actual','Desired');
set(l,'interpreter','latex')
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\Omega_{2}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,ang_vel(:,2),'b');
plot(t,ang_vel_des(2,:),'r');
set(gca,'FontName',fontname,'FontSize',fontsize);

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\Omega_{3}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
grid on;hold on
plot(t,ang_vel(:,3),'b');
plot(t,ang_vel_des(3,:),'r');

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot the disturbance estimate
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

hold all
grid on
% title('$\bar{\Delta}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('$t (sec)$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('$\bar{\Delta}$','interpreter','latex','FontName',fontname,'FontSize',fontsize)
plot(t,delta_est);

set(gca,'FontName',fontname,'FontSize',fontsize);

% plot attitude on unit sphere
% create a sphere
fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

set(fig_handle(fig_index),'Color','w');

hold all

[sph.x, sph.y, sph.z]=sphere(100);
% radius of cone at unit length
% loop over the number of constraints and draw them all
h_cyl = zeros(constants.num_con,1);
for ii = 1:constants.num_con
    
    H = cos(constants.con_angle(ii)); %// height
    R = sin(constants.con_angle(ii)); %// chord length
    N = 100; %// number of points to define the circumference
    [cyl.x, cyl.y, cyl.z] = cylinder([0 R], N);
    cyl.z = cyl.z*H;
    % calculate the rotation matrix to go from pointing in e3 direction to
    % along the con unit vector
    if sum(constants.con(:,ii) == [0;0;1]) == 3
        dcm = eye(3,3);
    elseif sum(constants.con(:,ii) == [0;0;-1]) == 3
        dcm = ROT1(pi);
    else
    k_hat = cross([0;0;1],constants.con(:,ii));
    angle = acos(dot([0;0;1],constants.con(:,ii)));
    dcm = eye(3,3) + hat_map(k_hat) + hat_map(k_hat)*hat_map(k_hat)*(1-cos(angle))/sin(angle)^2;
    end
    
    cyl_open = dcm*[cyl.x(2,:);cyl.y(2,:);cyl.z(2,:)];
    cyl.x(2,:) = cyl_open(1,:);
    cyl.y(2,:) = cyl_open(2,:);
    cyl.z(2,:) = cyl_open(3,:);
    
    
    h_cyl(ii) = surf(cyl.x,cyl.y,cyl.z);
end
h_sph=surf(sph.x,sph.y,sph.z);

set(h_sph,'LineStyle','none','FaceColor',0.8*[1 1 1],...
    'FaceLighting','gouraud','AmbientStrength',0.5,...
    'Facealpha',0.3,'Facecolor',[0.8 0.8 0.8]);
set(h_cyl,'Linestyle','none',...
    'FaceLighting','gouraud','AmbientStrength',0.5,...
    'Facealpha',0.5,'Facecolor','red');

light('Position',[0 0 100],'Style','infinite');
material dull;
axis equal;
axis off
xlabel('x')
ylabel('y')
zlabel('z')
% convert the body fixed vector to the inertial frame
sen_inertial = zeros(length(tspan),3);

for ii = 1:length(tspan)
   sen_inertial(ii,:) = (R_b2i(:,:,ii)*constants.sen)'; 
end
sen_inertial_start = constants.R0*constants.sen;
sen_inertial_end = constants.Rd*constants.sen;
% plot path of body vector in inertial frame
plot3(sen_inertial_start(1),sen_inertial_start(2),sen_inertial_start(3),'go','markersize',10,'linewidth',2)
plot3(sen_inertial_end(1),sen_inertial_end(2),sen_inertial_end(3),'gx','markersize',10,'linewidth',2)
plot3(sen_inertial(:,1),sen_inertial(:,2),sen_inertial(:,3),'b','linewidth',3)

% plot inertial frame
line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);
view(3)

% plot the translation of the vehicle

fig_index = fig_index+1;
set(0, 'CurrentFigure', fig_handle(fig_index)) % attitude error vector

hold all
grid on
title('Position','interpreter','latex','FontName',fontname,'FontSize',fontsize)
xlabel('x','interpreter','latex','FontName',fontname,'FontSize',fontsize)
ylabel('y','interpreter','latex','FontName',fontname,'FontSize',fontsize)
zlabel('z','interpreter','latex','FontName',fontname,'FontSize',fontsize)
axis equal

% plot the obstacle as a ellipsoid
plot_gaussian_ellipsoid(constants.xo, constants.N);

body_x0 = [1;0;0];
body_y0 = [0;1;0];
body_z0 = [0;0;1];

plot3(pos(:,1),pos(:,2),pos(:,3));

for ii = 1:length(t)
    % rotate using the direction cosine matrix
    body_x = R_b2i(:,:,ii)*body_x0;
    body_y = R_b2i(:,:,ii)*body_y0;
    body_z = R_b2i(:,:,ii)*body_z0;
    % translate at each time step to the current position
    % draw axes centered at the body
    line([pos(ii,1) pos(ii,1)+body_x(1)],[pos(ii,2) pos(ii,2)+body_x(2)],[pos(ii,3) pos(ii,3)+body_x(3)],'color','r','linewidth',3);
    line([pos(ii,1) pos(ii,1)+body_y(1)],[pos(ii,2) pos(ii,2)+body_y(2)],[pos(ii,3) pos(ii,3)+body_y(3)],'color','g','linewidth',3);
    line([pos(ii,1) pos(ii,1)+body_z(1)],[pos(ii,2) pos(ii,2)+body_z(2)],[pos(ii,3) pos(ii,3)+body_z(3)],'color','b','linewidth',3);
    
end