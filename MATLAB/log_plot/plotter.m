clc; clear;

% CSV 파일 읽기
filename = '20241207_0409_logdata_withwheel_torque_speed_iqvalue_offset10_sampletime0.2s_10to1000.csv'; % CSV 파일 이름
rawData = readmatrix(filename); % CSV 데이터를 행렬로 읽기

% 변환 상수
iq_to_actual = 33 / 2048; % Iq 변환 계수 (raw to actual current)
speed_to_rads = pi / 1800; % 1 dps/LSB -> rad/s 변환 계수
encoder_to_degrees = 2*pi/65536;

% 데이터 분리
timeStamp = rawData(:, 1);  % 첫 번째 열: TimeStamp

% Motor 1 데이터
motor1_input = rawData(:, 4);     % Motor 1 TorqueValue
motor1_totalAngle = rawData(:, 5) * encoder_to_degrees; % Motor 1 TotalAngle 실제 값으로 변환
motor1_iqcurrent = rawData(:, 6) * iq_to_actual;  % Motor 1 iqcurrentValue를 실제 값으로 변환
motor1_speed = rawData(:, 7)* speed_to_rads;       % Motor 1 Speed를 rad/s로 변환

% Motor 2 데이터
motor2_input = rawData(:, 10);     % Motor 2 TorqueValue
motor2_totalAngle = rawData(:, 11) * encoder_to_degrees; % Motor 2 TotalAngle을 실제 값으로 변환
motor2_iqcurrent = rawData(:, 12) * iq_to_actual; % Motor 2 iqcurrentValue를 실제 값으로 변환
motor2_speed = rawData(:, 13)* speed_to_rads;      % Motor 2 Speed를 rad/s로 변환

% Torque Constant (Nm/A)
torque_constant = 0.07;

calculated_motor1_torque = motor1_iqcurrent * torque_constant; % Motor 1 Calculated Torque
calculated_motor2_torque = motor2_iqcurrent * torque_constant; % Motor 2 Calculated Torque

% Sampling time 계산 (TimeStamp는 밀리초 단위라고 가정)
dt = diff(timeStamp) / 1000; % 초 단위로 변환

% 각속도 계산 (Angular Velocity)
motor1_velocity = diff(motor1_totalAngle) ./ dt; % Motor 1 각속도
motor2_velocity = diff(motor2_totalAngle) ./ dt; % Motor 2 각속도

% 각가속도 계산 (Angular Acceleration)
motor1_acceleration = diff(motor1_velocity) ./ dt(1:end-1); % Motor 1 각가속도
motor2_acceleration = diff(motor2_velocity) ./ dt(1:end-1); % Motor 2 각가속도

% Inertia
left_Inertia = 0.00072399902807526; % kg*m^2
right_Inertia = 0.00064074279507983; % kg*m^2

inertia_motor1_torque = left_Inertia * motor1_acceleration;
inertia_motor2_torque = right_Inertia * motor2_acceleration;

% Speed를 사용하여 Acceleration 계산
motor1_acceleration_from_speed = diff(motor1_speed) ./ dt; % Motor 1 Acceleration from Speed
motor2_acceleration_from_speed = diff(motor2_speed) ./ dt; % Motor 2 Acceleration from Speed

% 시간축 조정
timeStamp_velocity = timeStamp(1:end-1); % Velocity 시간축
timeStamp_acceleration = timeStamp(1:end-2); % Acceleration 시간축
timeStamp_acceleration_speed = timeStamp(1:end-1); % Speed 기반 Acceleration 시간축

% Butterworth 필터 설계 (저역 통과 필터)
fs = 1 / mean(dt); % 샘플링 주파수 (Hz)
cutoff_freq = 10; % 차단 주파수 (Hz)
[b, a] = butter(4, cutoff_freq / (fs / 2), 'low'); % 4차 Butterworth 필터 설계

% 필터 적용
filtered_motor1_iqcurrent = filtfilt(b, a, motor1_iqcurrent); % Motor 1 iqcurrent 필터
filtered_motor2_iqcurrent = filtfilt(b, a, motor2_iqcurrent); % Motor 2 iqcurrent 필터
filtered_motor1_acceleration = filtfilt(b, a, motor1_acceleration); % Motor 1 각가속도 필터
filtered_motor2_acceleration = filtfilt(b, a, motor2_acceleration); % Motor 2 각가속도 필터
filtered_motor1_acceleration_speed = filtfilt(b, a, motor1_acceleration_from_speed); % Motor 1 Speed 기반 가속도 필터
filtered_motor2_acceleration_speed = filtfilt(b, a, motor2_acceleration_from_speed); % Motor 2 Speed 기반 가속도 필터
filtered_calculated_motor1_torque = filtfilt(b, a, calculated_motor1_torque); % Motor 1 Calculated Torque 필터
filtered_calculated_motor2_torque = filtfilt(b, a, calculated_motor2_torque); % Motor 2 Calculated Torque 필터
filtered_inertia_motor1_torque = filtfilt(b, a, inertia_motor1_torque); % Motor 1 Inertia Torque 필터
filtered_inertia_motor2_torque = filtfilt(b, a, inertia_motor2_torque); % Motor 2 Inertia Torque 필터

% Plot 데이터 비교
figure;

% Total Angle 비교
subplot(5, 2, 1);
plot(timeStamp, motor1_totalAngle, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp, motor2_totalAngle, 'b', 'DisplayName', 'Motor 2');
title('Total Angle');
xlabel('TimeStamp');
ylabel('Total Angle');
legend('show'); hold off;

% Angular Velocity 비교
subplot(5, 2, 3);
plot(timeStamp_velocity, motor1_velocity, 'g', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp_velocity, motor2_velocity, 'm', 'DisplayName', 'Motor 2');
title('Angular Velocity');
xlabel('TimeStamp');
ylabel('Angular Velocity (units/s)');
legend('show'); hold off;

% Angular Acceleration 비교
subplot(5, 2, 5);
plot(timeStamp_acceleration, filtered_motor1_acceleration, 'k', 'DisplayName', 'Motor 1 (Filtered)'); hold on;
plot(timeStamp_acceleration, filtered_motor2_acceleration, 'c', 'DisplayName', 'Motor 2 (Filtered)');
title('Angular Acceleration (Filtered)');
xlabel('TimeStamp');
ylabel('Angular Acceleration (units/s^2)');
legend('show'); hold off;

% Speed 비교
subplot(5, 2, 7);
plot(timeStamp, motor1_speed, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp, motor2_speed, 'b', 'DisplayName', 'Motor 2');
title('Speed');
xlabel('TimeStamp');
ylabel('Speed');
legend('show'); hold off;

% Speed 기반 Acceleration 비교
subplot(5, 2, 2);
plot(timeStamp_acceleration_speed, filtered_motor1_acceleration_speed, 'k', 'DisplayName', 'Motor 1 (Filtered)'); hold on;
plot(timeStamp_acceleration_speed, filtered_motor2_acceleration_speed, 'c', 'DisplayName', 'Motor 2 (Filtered)');
title('Acceleration from Speed (Filtered)');
xlabel('TimeStamp');
ylabel('Acceleration');
legend('show'); hold off;

% Torque 비교
subplot(5, 2, 4);
plot(timeStamp, motor1_input, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp, motor2_input, 'b', 'DisplayName', 'Motor 2');
title('Input');
xlabel('TimeStamp');
ylabel('Input');
legend('show'); hold off;

% iqcurrentValue 비교
subplot(5, 2, 6);
plot(timeStamp, filtered_motor1_iqcurrent, 'r', 'DisplayName', 'Motor 1 (Filtered)'); hold on;
plot(timeStamp, filtered_motor2_iqcurrent, 'b', 'DisplayName', 'Motor 2 (Filtered)');
title('IqCurrent (Filtered)');
xlabel('TimeStamp');
ylabel('IqCurrent');
legend('show'); hold off;

% Calculated Torque 비교 (IqCurrent 기반)
subplot(5, 2, 8);
plot(timeStamp, filtered_calculated_motor1_torque, 'r', 'DisplayName', 'Motor 1 (Calculated)'); hold on;
plot(timeStamp, filtered_calculated_motor2_torque, 'b', 'DisplayName', 'Motor 2 (Calculated)');
title('Calculated Torque (Filtered)');
xlabel('TimeStamp');
ylabel('Torque (Nm)');
legend('show'); hold off;

% Inertia Torque 비교 (한 Figure에 두 Motor 함께 플롯)
subplot(5, 1, 5);
plot(timeStamp_acceleration, filtered_inertia_motor1_torque, 'r', 'DisplayName', 'Motor 1 (Inertia Torque)'); hold on;
plot(timeStamp_acceleration, filtered_inertia_motor2_torque, 'b', 'DisplayName', 'Motor 2 (Inertia Torque)');
title('Inertia Torque (Filtered)');
xlabel('TimeStamp');
ylabel('Torque (Nm)');
legend('show'); hold off;
