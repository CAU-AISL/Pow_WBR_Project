clc; clear;
close all;
format long;

addpath("lib\");
addpath("log_plot\");
load('dynamic_properties.mat');
load('dynamics_functions.mat');

% CSV 파일 읽기
filename = '20250112_logdata_EKF_LowGain_withaccLPF_VICON_005.csv'; % CSV 파일 이름

data = readtable(filename);

Ts = 0.008;

x_lim = [10, 40];

psi_dot_hat = data.psi_dot_hat;
theta_hat = data.theta_hat;
theta_dot_hat = data.theta_dot_hat;
v_hat = data.v_hat;

% EKF Parameters
P_init = eye(4)*1; % 초기 추정 오차 공분산 행렬
R_cov = diag([4e-1, 4, 4e-1,...
              1e-2, 1e-4, 1e-2,...
              0, 0]); % Sensor noise Covariance Matrix
Q_cov = diag([0, 1, 1, 1]); % Processor noise Covariance Matrix

% Estimator
x_hat_init = [0; 0; 0; 0];
x_hat = x_hat_init;

model = Pol(properties, dynamic_functions);
Estimator = EKF_c(x_hat_init, P_init, Q_cov, R_cov, model, Ts);

% extract data
timeStamp = data.TimeStamp / 10^3;

h_d = data.h_d;
theta_d = data.theta_d;
v_d = data.v_d;
psi_dot_d = data.psi_dot_d;

acc_x_imu = data.acc_x; % m/s^2
acc_y_imu = data.acc_y; % m/s^2
acc_z_imu = data.acc_z; % m/s^2
gyr_x_imu = data.gyr_x; % rad/s
gyr_y_imu = data.gyr_y; % rad/s
gyr_z_imu = data.gyr_z; % rad/s
theta_dot_LW = data.theta_dot_LW;
theta_dot_RW = data.theta_dot_RW;

z = [acc_x_imu'; acc_y_imu'; acc_z_imu';...
    gyr_x_imu'; gyr_y_imu'; gyr_z_imu'; ...
    theta_dot_RW'; theta_dot_LW'];

% =====================================================

% z(1,:) = sqrt(z(1,:).^2+z(2,:).^2);
% z(2,:) = z(2,:)*0;

% =====================================================

tau_LW = data.tau_LW;
tau_RW = data.tau_RW;
u = [tau_RW'; tau_LW'];

x_hat_cal = zeros(4,length(timeStamp));
x_pred = x_hat_cal;
P_pred_log = x_hat_cal;
P_log = x_hat_cal;
h_obs = zeros(8,length(timeStamp));
for i = 1:length(timeStamp)
    if i <= 2
        u_prevprev = [0;0];
    else
        u_prevprev = u(:,i-2);
        % u_prevprev = [0;0];
    end
    h = h_d(i);

    Estimator.predict(u_prevprev, h);
    x_pred(:,i) = Estimator.x;
    P_pred_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));
    Estimator.update(z(:,i));
    x_hat_cal(:,i) = Estimator.x;
    P_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));
    
    % model.setState(Estimator.x,u_prevprev,h);
    % model.calculateDynamics();
    % h_obs(:,i) = model.get_measurement_truth();
    
    h_obs(:,i) = Estimator.h_obs;
end

error = h_obs-z; 
for i = 1 : 8
    var(error(i,:))
end

% Plot 데이터 비교
figure('units','normalized','outerposition',[0 0 1 1]);


upper_bound = x_hat_cal(1,:) + 1.96 * P_log(1,:);
lower_bound = x_hat_cal(1,:) - 1.96 * P_log(1,:);
% upper_bound = x_pred(1,:) + 1.96 * P_pred_log(1,:);
% lower_bound = x_pred(1,:) - 1.96 * P_pred_log(1,:);
subplot(2, 3, 1);
plot(timeStamp, rad2deg(theta_hat), 'r', 'DisplayName', '\theta_{hat}'); hold on;
plot(timeStamp, rad2deg(x_hat_cal(1,:)), 'b--', 'DisplayName', '\theta_{hat}(cal)'); 
% plot(timeStamp, rad2deg(x_pred(1,:)), 'g.', 'DisplayName', '\theta_{pred}(cal)'); 
plot(timeStamp, rad2deg(theta_d), 'k', 'DisplayName', '\theta_{d}');
fill([timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title('pitch angle');
xlabel('TimeStamp');
ylabel('deg');
xlim(x_lim);
legend('show'); hold off;

upper_bound = x_hat_cal(2,:) + 1.96 * P_log(2,:);
lower_bound = x_hat_cal(2,:) - 1.96 * P_log(2,:);
% upper_bound = x_pred(2,:) + 1.96 * P_pred_log(2,:);
% lower_bound = x_pred(2,:) - 1.96 * P_pred_log(2,:);
subplot(2, 3, 2);
hold on;
plot(timeStamp, rad2deg(theta_dot_hat), 'r', 'DisplayName', 'd\theta_{hat}'); 
plot(timeStamp, rad2deg(x_hat_cal(2,:)), 'b--', 'DisplayName', 'd\theta_{hat}(cal)'); 
% plot(timeStamp, rad2deg(x_pred(2,:)), 'g.', 'DisplayName', 'd\theta_{pred}(cal)'); 
plot(timeStamp, zeros(length(timeStamp),1), 'k', 'DisplayName', 'd\theta_{d}');
fill([timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title('pitch angular rate');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;

upper_bound = x_hat_cal(3,:) + 1.96 * P_log(3,:);
lower_bound = x_hat_cal(3,:) - 1.96 * P_log(3,:);
% upper_bound = x_pred(3,:) + 1.96 * P_pred_log(3,:);
% lower_bound = x_pred(3,:) - 1.96 * P_pred_log(3,:);
subplot(2, 3, 4);
hold on;
plot(timeStamp, v_hat, 'r', 'DisplayName', 'v_{hat}'); 
plot(timeStamp, x_hat_cal(3,:), 'b--', 'DisplayName', 'v_{hat}(cal)'); 
% plot(timeStamp, x_pred(3,:), 'g.', 'DisplayName', 'v_{pred}(cal)'); 
plot(timeStamp, v_d, 'k', 'DisplayName', 'v_{d}');
fill([timeStamp', fliplr(timeStamp')], ...
     [upper_bound, fliplr(lower_bound)], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title('velocity');
xlabel('TimeStamp');
ylabel('m/s');
xlim(x_lim);
legend('show'); hold off;

upper_bound = x_hat_cal(4,:) + 1.96 * P_log(4,:);
lower_bound = x_hat_cal(4,:) - 1.96 * P_log(4,:);
% upper_bound = x_pred(4,:) + 1.96 * P_pred_log(4,:);
% lower_bound = x_pred(4,:) - 1.96 * P_pred_log(4,:);
subplot(2, 3, 5);
hold on;
plot(timeStamp, rad2deg(psi_dot_hat), 'r', 'DisplayName', 'd\psi_{hat}'); 
plot(timeStamp, rad2deg(x_hat_cal(4,:)), 'b--', 'DisplayName', 'd\psi_{hat}(cal)');
% plot(timeStamp, rad2deg(x_pred(4,:)), 'g.', 'DisplayName', 'd\psi_{pred}(cal)'); 
plot(timeStamp, rad2deg(psi_dot_d), 'k', 'DisplayName', 'd\psi_{d}');
fill([timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title('yaw angular rate');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 3, 3);
plot(timeStamp, tau_RW, 'g', 'DisplayName', '\tau_{RW}'); hold on;
title('Right Wheel Input');
xlabel('TimeStamp');
ylabel('Nm');
xlim(x_lim);

subplot(2, 3, 6);
plot(timeStamp, tau_LW, 'g', 'DisplayName', '\tau_{LW}'); hold on;
title('Left Wheel Input');
xlabel('TimeStamp');
ylabel('Nm');
xlim(x_lim);

figure('units','normalized','outerposition',[0 0 1 1]);
% total_acc = sqrt(acc_x_imu.^2+acc_y_imu.^2+acc_z_imu.^2);
subplot(2, 4, 1);
hold on;
plot(timeStamp, z(1,:), 'r', 'DisplayName', 'acc_x'); 
plot(timeStamp, h_obs(1,:), 'g', 'DisplayName', 'acc_x_obs'); 
title('IMU Acceleration');
xlabel('TimeStamp');
ylabel('m/s^2');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 2);
hold on;
plot(timeStamp, z(2,:), 'r', 'DisplayName', 'acc_y');
plot(timeStamp, h_obs(2,:), 'g', 'DisplayName', 'acc_y_obs'); 
title('IMU Acceleration');
xlabel('TimeStamp');
ylabel('m/s^2');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 3);
hold on;
plot(timeStamp, z(3,:), 'r', 'DisplayName', 'acc_z');
plot(timeStamp, h_obs(3,:), 'g', 'DisplayName', 'acc_z_obs'); 
title('IMU Acceleration');
xlabel('TimeStamp');
ylabel('m/s^2');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 5);
hold on;
plot(timeStamp, rad2deg(z(4,:)), 'r', 'DisplayName', 'gyr_x'); 
plot(timeStamp, rad2deg(h_obs(4,:)), 'g', 'DisplayName', 'gyr_x_obs'); 
title('IMU Angular Velocity');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;


subplot(2, 4, 6);
hold on;
plot(timeStamp, rad2deg(z(5,:)), 'r', 'DisplayName', 'gyr_y');
plot(timeStamp, rad2deg(h_obs(5,:)), 'g', 'DisplayName', 'gyr_y_obs'); 
title('IMU Angular Velocity');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 7);
hold on;
plot(timeStamp, rad2deg(z(6,:)), 'r', 'DisplayName', 'gyr_z');
plot(timeStamp, rad2deg(h_obs(6,:)), 'g', 'DisplayName', 'gyr_z_obs'); 
title('IMU Angular Velocity');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 4);
hold on;
plot(timeStamp, rad2deg(z(7,:)), 'r', 'DisplayName', 'd\theta_{RW}');
plot(timeStamp, rad2deg(h_obs(7,:)), 'g', 'DisplayName', 'd\theta_{RW}_obs'); 
title('Right Wheel speed');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;

subplot(2, 4, 8);
hold on;
plot(timeStamp, rad2deg(z(8,:)), 'r', 'DisplayName', 'd\theta_{LW}');
plot(timeStamp, rad2deg(h_obs(8,:)), 'g', 'DisplayName', 'd\theta_{LW}_obs'); 
title('Left Wheel speed');
xlabel('TimeStamp');
ylabel('deg/s');
xlim(x_lim);
legend('show'); hold off;
