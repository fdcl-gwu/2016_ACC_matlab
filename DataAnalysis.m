close all;
clear all;
clc

t_start_test =20 ;

% Format from C-Code:
%{
int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc_CADS,"%E... ",
                    NewDataVicon, NewDataIMU, del_t_v, del_t_IMU,// 1-4
                    x_e[0], x_e[1], x_e[2],// 5-7
                    xd[0], xd[1], xd[2],// 8-10
                    v_e[0], v_e[1], v_e[2],// 11-13
                    xd_dot[0],  xd_dot[1],  xd_dot[2],// 14-16
                    R_eb[0][0], R_eb[0][1], R_eb[0][2],// 17-19
                    R_eb[1][0], R_eb[1][1], R_eb[1][2],// 20-22
                    R_eb[2][0], R_eb[2][1], R_eb[2][2],// 23-25
                    Rd[0][0], Rd[0][1], Rd[0][2],// 26-28
                    Rd[1][0], Rd[1][1], Rd[1][2],// 29-31
                    Rd[2][0], Rd[2][1], Rd[2][2],// 32-34
                    W_b[0], W_b[1], W_b[2],// 35-37
                    Wd[0], Wd[1], Wd[2]// 38-40
                    );

%}

% Load data
[filename, pathname] = uigetfile('.txt');
[data] = load(strcat(pathname,filename));

[readings, elements] = size(data);

% Mean time steps
dt_v_med = median(data(:,3)); dt_IMU_med = median(data(:,4));
disp(['Vicon mean time step: ', num2str(dt_v_med), ' sec.']);
disp(['IMU mean time step: ', num2str(dt_IMU_med), ' sec.']);

% Indices where the sensors updated
IndVicon = find(data(:,1)+1); numIndVicon = numel(IndVicon);
IndIMU = find(data(:,2)+1);   numIndIMU = numel(IndIMU);

% Vicon analysis
t_v = zeros(1,numIndVicon);
x_e = zeros(3,numIndVicon);
xd = zeros(3,numIndVicon);
err_x = zeros(3,numIndVicon);
v_e = zeros(3,numIndVicon);
xd_dot = zeros(3,numIndVicon);
err_xdot = zeros(3,numIndVicon);
R_eb_9by1 = zeros(9,numIndVicon);
Rd_9by1 = zeros(9,numIndVicon);
err_R = zeros(3,numIndVicon);
config_err_fun_R = zeros(1,numIndVicon);
W_Vicon = zeros(3,numIndVicon);

for k = 1:numIndVicon
    % Time
    if k == 1
        t_v(k) = 0;
    else
        t_v(k) = t_v(k-1)+data(IndVicon(k),3);
    end
    dtViconSave(k) = data(IndVicon(k),3);
    % Position
    x_e(:,k) = data(IndVicon(k),5:7);
    xd(:,k) = data(IndVicon(k),8:10);
    err_x(:,k) = x_e(:,k)-xd(:,k);
    
    % Velocity
    v_e(:,k) = data(IndVicon(k),11:13);
    xd_dot(:,k) = data(IndVicon(k),14:16);
    err_xdot(:,k) = v_e(:,k)-xd_dot(:,k);
    
    % Attitude
    R_eb_9by1(:,k) = data(IndVicon(k),17:25);
    Rd_9by1(:,k) =   data(IndVicon(k),26:34);
    R_eb = reshape(R_eb_9by1(:,k),3,3)';
    Rd = reshape(Rd_9by1(:,k),3,3)';
    err_R(:,k) = 1/2*vee_map(Rd'*R_eb-R_eb'*Rd);
    config_err_fun_R(k) = 1/2*trace(eye(3)-Rd'*R_eb);
    if k ~= 1
        W_Vicon(:,k) = vee_map(R_eb'*(R_eb-R_eb_last)/data(IndVicon(k),3));
    end
    R_eb_last = R_eb;

end

% IMU Analysis
t_IMU = zeros(1,numIndIMU);
W_b = zeros(3,numIndIMU);
Wd = zeros(3,numIndIMU);
err_W = zeros(3,numIndIMU);

for k = 1:numIndIMU
    
    % Time
    if k == 1
        t_IMU(k) = 0;
    else
        t_IMU(k) = t_IMU(k-1)+data(IndIMU(k),4);
    end
    
    % Angular Velocity
    W_b(:,k) = data(IndIMU(k),35:37);
    Wd(:,k) = data(IndIMU(k),38:40);
    err_W(:,k) = W_b(:,k)-Wd(:,k);

end

% Adjust t_v for the time when only the motors are working
t = t_v-t_start_test;
t_IMU = t_IMU-t_start_test;

t_end_test = max([t, t_IMU]);


load_experiment_constants
tspan = t;
% save states into variable consistent with simulation
R_b2i = zeros(3,3,length(t));
u_f = zeros(3,length(t));
u_m = zeros(3,length(t));
R_des = zeros(3,3,length(t));
ang_vel_des = Wd;
ang_vel_dot_des = zeros(3,length(t));
Psi = zeros(length(t),1);
err_att = zeros(3,length(t));
err_vel = zeros(3,length(t));

ang_vel = pchip(t_IMU,W_b,t);
delta_est = zeros(3,length(t));
for ii = 1:length(t)
   R_b2i(:,:,ii) = reshape(R_eb_9by1(:,ii),3,3)'; 
   R_des(:,:,ii) = reshape(Rd_9by1(:,ii),3,3)';
   
   Rb2i_curr = R_b2i(:,:,ii);
   w_curr = ang_vel(:,ii);
   delta_est_curr = delta_est(:,ii);
   state_curr = [Rb2i_curr(:);w_curr; delta_est_curr];
   [u_f(:,ii), u_m(:,ii), R_des(:,:,ii), ang_vel_des(:,ii), ang_vel_dot_des(:,ii), Psi(ii), err_att(:,ii), err_vel(:,ii)] ...
    = controller(t(ii),state_curr, constants);
end

% adjust the data based on the start time (remove all values before 0)
index = t>=0;
t = t(index);
R_b2i = R_b2i(:,:,index);
R_des = R_des(:,:,index);
ang_vel = ang_vel(:,index);
u_m = u_m(:,index);
Psi = Psi(index);
err_att = err_att(:,index);
err_vel = err_vel(:,index);
% use simulation plotting tools to plot
plot_experiment_outputs

% % Plotting
% set(0,'DefaultAxesFontSize',12);

% 
% figure;% Position
% subplot(3,1,1);
% plot(t_v, x_e(1,:), 'b-', ...
%      t_v, xd(1,:), 'k-');
% ylabel('$$x_1\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,2);
% plot(t_v, x_e(2,:), 'b-', ...
%      t_v, xd(2,:), 'k-');
% ylabel('$$x_2\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,3);
% plot(t_v, x_e(3,:), 'b-', ...
%      t_v, xd(3,:), 'k-');
% ylabel('$$x_3\ ({\bf m})$$','interpreter','latex');
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Position Error
% subplot(3,1,1);
% plot(t_v, err_x(1,:), 'k-');
% ylabel('$$e_{x_1}\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,2);
% plot(t_v, err_x(2,:), 'k-');
% ylabel('$$e_{x_2}\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,3);
% plot(t_v, err_x(3,:), 'k-');
% ylabel('$$e_{x_3}\ ({\bf m})$$','interpreter','latex');
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Velocity
% subplot(3,1,1);
% plot(t_v, v_e(1,:), 'b-', ...
%      t_v, xd_dot(1,:), 'k-');
% ylabel('$$\dot x_1\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,2);
% plot(t_v, v_e(2,:), 'b-', ...
%      t_v, xd_dot(2,:), 'k-');
% ylabel('$$\dot x_2\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,3);
% plot(t_v, v_e(3,:), 'b-', ...
%      t_v, xd_dot(3,:), 'k-');
% ylabel('$$\dot x_3\ ({\bf m})$$','interpreter','latex');
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Velocity Error
% subplot(3,1,1);
% plot(t_v, err_xdot(1,:), 'k-');
% ylabel('$$e_{\dot x_1}\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,2);
% plot(t_v, err_xdot(2,:), 'k-');
% ylabel('$$e_{\dot x_2}\ ({\bf m})$$','interpreter','latex');
% 
% subplot(3,1,3);
% plot(t_v, err_xdot(3,:), 'k-');
% ylabel('$$e_{\dot x_3}\ ({\bf m})$$','interpreter','latex');
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Attitude
% for i = 1:9
%     subplot(3,3,i);
%     plot(t_v, R_eb_9by1(i,:), 'b-', ...
%          t_v, Rd_9by1(i,:), 'k-');
% end
% 
% figure;% Attitude Error
% subplot(3,1,1);
% plot(t_v, err_R(1,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{R,1}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(err_R(1,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,2);
% plot(t_v, err_R(2,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{R,2}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(err_R(2,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,3);
% plot(t_v, err_R(3,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{R,3}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(err_R(3,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Attitude Configuration Error Function
% plot(t_v, config_err_fun_R, 'k-', 'LineWidth', 2);
% ylabel('$$\text{\frac{1}{2}tr}(I-R_d^TR)$$','interpreter','latex');
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% axis([0, t_end_test, 0, 1.1*max(config_err_fun_R)]);
% 
% figure;% Angular Velocity
% subplot(3,1,1);
% plot(t_v, W_Vicon(1,:), 'b-', ...
%      t_IMU, W_b(1,:), 'k-');%, 'LineWidth', 2);
% ylabel('$$\Omega_1\ ({\bf rad/sec})$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs([W_Vicon(1,:),W_b(1,:)]));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,2);
% plot(t_v, W_Vicon(2,:), 'b-', ...
%      t_IMU, W_b(2,:), 'k-');%, 'LineWidth', 2);
% ylabel('$$\Omega_1\ ({\bf rad/sec})$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs([W_Vicon(1,:),W_b(1,:)]));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,3);
% plot(t_v, W_Vicon(3,:), 'b-', ...
%      t_IMU, W_b(3,:), 'k-');%, 'LineWidth', 2);
% ylabel('$$\Omega_1\ ({\bf rad/sec})$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs([W_Vicon(1,:),W_b(1,:)]));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');
% 
% figure;% Angular Velocity Error
% subplot(3,1,1);
% plot(t_IMU, err_W(1,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{\Omega_1}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(W_b(1,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,2);
% plot(t_IMU, err_W(2,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{\Omega_2}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(W_b(2,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% subplot(3,1,3);
% plot(t_IMU, err_W(3,:), 'k-', 'LineWidth', 2);
% ylabel('$$e_{\Omega_3}$$','interpreter','latex');
% y_axis_lim = 1.1*max(abs(W_b(3,:)));
% axis([0, t_end_test, -y_axis_lim, y_axis_lim]);
% 
% xlabel('$$t\ ({\bf sec})$$','interpreter','latex');


