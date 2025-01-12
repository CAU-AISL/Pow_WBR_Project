clc; clear;
% close all;
format long;

addpath("lib\");
addpath("log_plot\");
addpath("VICON_data\");

% CSV 파일 읽기
filename = '20250112_VICON_Test_005.csv'; % CSV 파일 이름
FPS = 250; % Hz
h = 0.2; % height (m)
r = 0.07215; % wheel radius

% CSV 파일 전체 읽기 (문자열 포함)
rawData = readcell(filename);

% 4행부터 데이터를 숫자 행렬로 변환
data = cell2mat(rawData(4:end, [1,3:end]));
data_len = size(data,1);

% Frame to Time
data(:,1) = data(:,1)/FPS;
% Deg to Rad
data(:,5:7) = deg2rad(data(:,5:7));

% Extract datas
timeStamp = data(:,1);
pos_raw_vecs = data(:,2:4)';


% Euler ZYX = roll-pitch-yaw
rolls = data(:,5);
pitchs = data(:,6);
yaws = data(:,7);

R_WC = zeros(3, 3, data_len);
R_CB = zeros(3, 3, data_len);

p_W_woco_vecs = zeros(3, data_len);
p_W_wobo_vecs = zeros(3, data_len);
for i = 1:data_len
    R_WC(:,:,i) = eulerZYXtoRotationMatrix(0,0,yaws(i));
    R_CB(:,:,i) = eulerZYXtoRotationMatrix(rolls(i),pitchs(i),0);
    p_W_woco_vecs(:,i) = pos_raw_vecs(:,i)-[0;0;r];
    p_W_wobo_vecs(:,i) = p_W_woco_vecs(:,i) + R_WC(:,:,i)*R_CB(:,:,i)*[0;0;h];
end

v_W_woco_vecs = gradient(p_W_woco_vecs) * FPS;
v_W_wobo_vecs = gradient(p_W_wobo_vecs) * FPS;

a_W_wobo_vecs = gradient(v_W_wobo_vecs) * FPS;
a_IMU = zeros(size(a_W_wobo_vecs));
v_C_woco_vecs = zeros(size(v_W_woco_vecs));
for i = 1 : data_len
    a_IMU(:,i) = (R_WC(:,:,i) * R_CB(:,:,i))' * (a_W_wobo_vecs(:,i) - [0;0;-9.80665]);
    v_C_woco_vecs(:,i) = R_WC(:,:,i)'*v_W_woco_vecs(:,i);
end

theta_trues = pitchs;
theta_dot_trues = gradient(theta_trues) * FPS;
v_trues = v_C_woco_vecs(1,:);
psi_dot_trues = gradient(yaws) * FPS;

% =================================== Figure 1 ============================
figure('units','normalized','outerposition',[0 0 1 1]);
sgtitle('Raw Position and Orientation', 'FontSize', 14, 'FontWeight', 'bold');

% Position 데이터
subplot(2,3,1);
plot(timeStamp, pos_raw_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Raw x position'); % 제목 추가
grid on;

subplot(2,3,2);
plot(timeStamp, pos_raw_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Raw y position'); % 제목 추가
grid on;

subplot(2,3,3);
plot(timeStamp, pos_raw_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Raw z position'); % 제목 추가
grid on;

% Orientation 데이터
subplot(2,3,4);
plot(timeStamp, rad2deg(rolls(:)), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Raw roll angle(\phi)'); % 제목 추가
grid on;

subplot(2,3,5);
plot(timeStamp, rad2deg(pitchs(:)), 'g', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Raw pitch angle(\theta)'); % 제목 추가
grid on;

subplot(2,3,6);
plot(timeStamp, rad2deg(yaws(:)), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Raw yaw angle (\psi)'); % 제목 추가
grid on;


% =================================== Figure 2 ============================
figure('units','normalized','outerposition',[0 0 1 1]);
sgtitle('Control position and Body position ', 'FontSize', 14, 'FontWeight', 'bold');

% Position 데이터
subplot(2,3,1);
plot(timeStamp, p_W_woco_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Control frame x position in World frame (p^{W}_{woco,x})'); % 제목 추가
grid on;

subplot(2,3,2);
plot(timeStamp, p_W_woco_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Control frame y position in World frame (p^{W}_{woco,y})'); % 제목 추가
grid on;

subplot(2,3,3);
plot(timeStamp, p_W_woco_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Control frame z position in World frame (p^{W}_{woco,z})'); % 제목 추가
grid on;

% Position 데이터
subplot(2,3,4);
plot(timeStamp, p_W_wobo_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Body frame x position in World frame (p^{W}_{wobo,x})'); % 제목 추가
grid on;

subplot(2,3,5);
plot(timeStamp, p_W_wobo_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Body frame y position in World frame (p^{W}_{wobo,y})'); % 제목 추가
grid on;

subplot(2,3,6);
plot(timeStamp, p_W_wobo_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Body frame z position in World frame (p^{W}_{wobo,z})'); % 제목 추가
grid on;

% =================================== Figure 3 ============================
figure('units','normalized','outerposition',[0 0 1 1]);
sgtitle('Control Velocity in World and Control frame ', 'FontSize', 14, 'FontWeight', 'bold');

% Velocity 데이터
subplot(2,3,1);
plot(timeStamp, v_W_woco_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame x velocity in World frame (v^{W}_{woco,x})'); % 제목 추가
grid on;

subplot(2,3,2);
plot(timeStamp, v_W_woco_vecs(2,:), 'g', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame y velocity in World frame (v^{W}_{woco,y})'); % 제목 추가
grid on;

subplot(2,3,3);
plot(timeStamp, v_W_woco_vecs(3,:), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame z velocity in World frame (v^{W}_{woco,z})'); % 제목 추가
grid on;

% Velocity 데이터
subplot(2,3,4);
plot(timeStamp, v_C_woco_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame x velocity in Control frame (v^{C}_{woco,x})'); % 제목 추가
grid on;

subplot(2,3,5);
plot(timeStamp, v_C_woco_vecs(2,:), 'g', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame y velocity in Control frame (v^{C}_{woco,y})'); % 제목 추가
grid on;

subplot(2,3,6);
plot(timeStamp, v_C_woco_vecs(3,:), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame z velocity in Control frame (v^{C}_{woco,z})'); % 제목 추가
grid on;

% =================================== Figure 4 ============================
figure('units','normalized','outerposition',[0 0 1 1]);
sgtitle('ture states ', 'FontSize', 14, 'FontWeight', 'bold');

% Velocity 데이터
subplot(2,2,1);
plot(timeStamp, rad2deg(theta_trues(:)), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('angle (deg)');
title('\theta_{true}'); % 제목 추가
grid on;

subplot(2,2,2);
plot(timeStamp, rad2deg(theta_dot_trues(:)), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('angle (deg)');
title('\theta^{dot}_{true}'); % 제목 추가
grid on;

subplot(2,2,3);
plot(timeStamp, v_trues(:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('\v_{true}'); % 제목 추가
grid on;

% Velocity 데이터
subplot(2,3,4);
plot(timeStamp, v_C_woco_vecs(1,:), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame x velocity in Control frame (v^{C}_{woco,x})'); % 제목 추가
grid on;

subplot(2,3,5);
plot(timeStamp, v_C_woco_vecs(2,:), 'g', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame y velocity in Control frame (v^{C}_{woco,y})'); % 제목 추가
grid on;

subplot(2,3,6);
plot(timeStamp, v_C_woco_vecs(3,:), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Control frame z velocity in Control frame (v^{C}_{woco,z})'); % 제목 추가
grid on;