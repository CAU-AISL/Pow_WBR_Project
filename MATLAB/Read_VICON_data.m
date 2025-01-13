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
r = 0.072; % wheel radius
time_cutter = [1.24, 30];

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
R_WB = zeros(3, 3, data_len);

p_W_woco_vecs = zeros(3, data_len);
p_W_wobo_vecs = zeros(3, data_len);
for i = 1:data_len
    R_WC(:,:,i) = eulerZYXtoRotationMatrix(0,0,yaws(i));
    R_CB(:,:,i) = eulerZYXtoRotationMatrix(rolls(i),pitchs(i),0);
    R_WB(:,:,i) = R_WC(:,:,i) * R_CB(:,:,i);
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

dR_WB = zeros(3,3,data_len);
for i = 1:3
    for j = 1:3
        temp_R = reshape(R_WB(i,j,:),1,data_len);
        % R_WB의 각 성분에 대해 gradient를 계산
        dR_WB(i, j, :) = gradient(temp_R) * FPS;
    end
end
w_IMU = zeros(3,data_len);
for i = 1:data_len
    w_IMU(:,i) = skew2vec(R_WB(:,:,i)'*dR_WB(:,:,i));
end

theta_trues = pitchs;
theta_dot_trues = gradient(theta_trues) * FPS;
v_trues = v_C_woco_vecs(1,:);
psi_dot_trues = gradient(yaws) * FPS;

    
screenSize = get(0, 'ScreenSize'); % 화면 크기 가져오기

% uifigure 생성 (전체 화면 크기로 설정)
fig = uifigure('Name', filename, 'Position', [0, 0, screenSize(3), screenSize(4)*0.97]);
tabGroup = uitabgroup(fig, 'Position', [0, 0, fig.Position(3), fig.Position(4)]);

% =================================== Figure 1 ============================
tab1 = uitab(tabGroup, 'Title', 'Raw Position and Orientation');
layout = tiledlayout(tab1, 2, 3, "TileSpacing", "compact");
title(layout,'Raw Position and Orientation', 'FontSize', 14, 'FontWeight', 'bold')

% Positions
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, pos_raw_vecs(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p_{x, raw}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, pos_raw_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p_{y, raw}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, pos_raw_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p_{z, raw}');
grid(ax, "on");
grid(ax, "minor");

% Orientations
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(rolls(:)), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, time_cutter);
title(ax, '\phi_{raw}'); 
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(pitchs(:)), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, time_cutter);
title(ax, '\theta_{raw}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(yaws(:)), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, time_cutter);
title(ax, '\psi_{raw}');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 2 ============================
tab2 = uitab(tabGroup, 'Title', 'Control position and Body position ');
layout = tiledlayout(tab2, 2, 3, "TileSpacing", "compact");
title(layout,'Control position and Body position ', 'FontSize', 14, 'FontWeight', 'bold')

% Positions
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, p_W_woco_vecs(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{woco,x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, p_W_woco_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{woco,y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, p_W_woco_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{woco,z}');
grid(ax, "on");
grid(ax, "minor");

% Orientations
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, p_W_wobo_vecs(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{wobo,x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
plot(ax, timeStamp, p_W_wobo_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{wobo,y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, p_W_wobo_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Position (m)');
xlim(ax, time_cutter);
title(ax, 'p^{W}_{wobo,z}');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 3 ============================
tab3 = uitab(tabGroup, 'Title', 'Control Velocity in World and Control frame ');
layout = tiledlayout(tab3, 2, 3, "TileSpacing", "compact");
title(layout, 'Control Velocity in World and Control frame', 'FontSize', 14, 'FontWeight', 'bold')

% Positions
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, v_W_woco_vecs(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{W}_{woco,x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, v_W_woco_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{W}_{woco,y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, v_W_woco_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{W}_{woco,z}');
grid(ax, "on");
grid(ax, "minor");

% Orientations
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, v_C_woco_vecs(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{C}_{woco,x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
plot(ax, timeStamp, v_C_woco_vecs(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{C}_{woco,y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, v_C_woco_vecs(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v^{C}_{woco,z}');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 4 ============================
tab4 = uitab(tabGroup, 'Title', 'True states');
layout = tiledlayout(tab4, 2, 2, "TileSpacing", "compact");
title(layout, 'True states', 'FontSize', 14, 'FontWeight', 'bold')

% Positions
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(theta_trues(:)), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, time_cutter);
title(ax, '\theta_{true}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(theta_dot_trues(:)), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, time_cutter);
title(ax, '\theta^{dot}_{true}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, v_trues(:), 'r', 'LineWidth', 1.5); 
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
title(ax, 'v_{true}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(psi_dot_trues(:)), 'r', 'LineWidth', 1.5); 
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, time_cutter);
title(ax, '\psi^{dot}_{true}');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 5 ============================
tab5 = uitab(tabGroup, 'Title', 'IMU trues');
layout = tiledlayout(tab5, 2, 3, "TileSpacing", "compact");
title(layout, 'IMU trues', 'FontSize', 14, 'FontWeight', 'bold')

% True Accelerometer values
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, a_IMU(1,:), 'r', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
title(ax, 'a_{IMU, x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, a_IMU(2,:), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
title(ax, 'a_{IMU, y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, a_IMU(3,:), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
title(ax, 'a_{IMU, z}');
grid(ax, "on");
grid(ax, "minor");

% True Gyroscope values
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(w_IMU(1,:)), 'r', 'LineWidth', 1.5); 
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, time_cutter);
title(ax, 'w_{IMU, x}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(w_IMU(2,:)), 'g', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, time_cutter);
title(ax, 'w_{IMU, y}');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(w_IMU(3,:)), 'b', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, time_cutter);
title(ax, 'w_{IMU, z}');
grid(ax, "on");
grid(ax, "minor");

