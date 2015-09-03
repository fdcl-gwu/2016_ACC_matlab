% 11 June 15
% plot data
close all

% % plot the position
% figure
% 
% subplot(3,1,1)
% title('Position of SC in inertial frame','interpreter','latex')
% xlabel('$t (sec)$','interpreter','latex')
% ylabel('$x (km)$','interpreter','latex')
% grid on;hold on
% plot(t,pos(:,1));
% 
% subplot(3,1,2)
% xlabel('$t (sec)$','interpreter','latex')
% ylabel('$y (km)$','interpreter','latex')
% grid on;hold on
% plot(t,pos(:,2));
% 
% subplot(3,1,3)
% xlabel('$t (sec)$','interpreter','latex')
% ylabel('$z (km)$','interpreter','latex')
% grid on;hold on
% plot(t,pos(:,3));

% plot the attitude error vector
figure % attitude error vector
subplot(3,1,1)
title('Attitude error vector','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{R1}$','interpreter','latex')
grid on;hold on
plot(t,err_att(1,:));

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{R2}$','interpreter','latex')
grid on;hold on
plot(t,err_att(2,:));

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{R3}$','interpreter','latex')
grid on;hold on
plot(t,err_att(3,:));

% plot the attitude error \Psi
figure % \Psi
title('$\Psi$ error ','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$\Psi$','interpreter','latex')
grid on;hold on
plot(t,Psi);



% plot the angular velocity error
figure 
subplot(3,1,1)
title('Angular velocity error vector','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{\Omega_1}$','interpreter','latex')
grid on;hold on
plot(t,err_vel(1,:));

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{\Omega_2}$','interpreter','latex')
grid on;hold on
plot(t,err_vel(2,:));

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$e_{\Omega_3}$','interpreter','latex')
grid on;hold on
plot(t,err_vel(3,:));

% plot the control input
figure 
subplot(3,1,1)
title('Control Input','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$u_{1}$','interpreter','latex')
grid on;hold on
plot(t,u_m(1,:));

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$u_{2}$','interpreter','latex')
grid on;hold on
plot(t,u_m(2,:));

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$u_{3}$','interpreter','latex')
grid on;hold on
plot(t,u_m(3,:));

% plot the desired adn actual angular velocities
figure

subplot(3,1,1)
title('Angular Velocity','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$\Omega_{1}$','interpreter','latex')
grid on;hold on
plot(t,ang_vel(:,1),'b');
plot(t,ang_vel_des(1,:),'r');
l=legend('Actual','Desired');
set(l,'interpreter','latex')

subplot(3,1,2)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$\Omega_{2}$','interpreter','latex')
grid on;hold on
plot(t,ang_vel(:,2),'b');
plot(t,ang_vel_des(2,:),'r');

subplot(3,1,3)
xlabel('$t (sec)$','interpreter','latex')
ylabel('$\Omega_{3}$','interpreter','latex')
grid on;hold on
plot(t,ang_vel(:,3),'b');
plot(t,ang_vel_des(3,:),'r');

% plot the disturbance estimate
figure
hold all
grid on
title('$\hat{\theta}$','interpreter','latex')
xlabel('$t (sec)$','interpreter','latex')
ylabel('$\theta$','interpreter','latex')
plot(t,theta_est);

% plot attitude on unit sphere
% create a sphere
h_fig=figure('color','w');
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
    'Facealpha',1,'Facecolor','red');

light('Position',[0 0 100],'Style','local');
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