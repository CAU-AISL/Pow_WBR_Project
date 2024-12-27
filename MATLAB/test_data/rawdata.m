clc; clear;
close all;

% CSV 파일 읽기
filename1 = 'logdata_squat_good.csv'; % CSV 파일 이름
% filename2 = 'logdata_squat_good_soft.csv'; % CSV 파일 이름
filename2 = 'logdata_forward_backward.csv'; % CSV 파일 이름
% filename2 = 'logdata_forward_backward_soft.csv'; % CSV 파일 이름
% filename1 = 'logdata_onstand_soft.csv'; % CSV 파일 이름
% filename2 = 'logdata_handstand_soft_2.csv'; % CSV 파일 이름

% filename1 = 'logdata_yawing.csv'; % CSV 파일 이름
% filename1 = 'logdata_yawing_soft.csv'; % CSV 파일 이름
% filename2 = 'logdata_yawing_soft_2.csv'; % CSV 파일 이름
plot_fft_data(filename1, filename2);
% rawData1 = readmatrix(filename1); % CSV 데이터를 행렬로 읽기
% rawData2 = readmatrix(filename2); % CSV 데이터를 행렬로 읽기
% 
% timeStamp1 = rawData1(:, 1)/1000;     % 1열: TimeStamp
% acc_x1 = rawData1(:, 2);              % 2열: acc_x
% acc_y1 = rawData1(:, 3);              % 3열: acc_y
% acc_z1 = rawData1(:, 4);              % 4열: acc_z
% gyr_x1 = rawData1(:, 5);              % 5열: gyr_x
% gyr_y1 = rawData1(:, 6);              % 6열: gyr_y
% gyr_z1 = rawData1(:, 7);              % 7열: gyr_z
% h_d1 = rawData1(:, 8);                % 8열: h_d
% psi_dot_d1 = rawData1(:, 9);          % 9열: psi_dot_d
% psi_dot_hat1 = rawData1(:, 10);       % 10열: psi_dot_hat
% tau_LW1 = rawData1(:, 11);            % 11열: tau_LW
% tau_RW1 = rawData1(:, 12);            % 12열: tau_RW
% theta_dot_hat1 = rawData1(:, 15);     % 15열: theta_dot_hat
% theta_eq1 = rawData1(:, 16);          % 16열: theta_eq
% theta_hat1 = rawData1(:, 17);         % 17열: theta_hat
% v_d1 = rawData1(:, 18);               % 18열: v_d
% v_hat1 = rawData1(:, 19);             % 19열: v_hat
% 
% timeStamp2 = rawData2(:, 1)/1000;     % 1열: TimeStamp
% acc_x2 = rawData2(:, 2);              % 2열: acc_x
% acc_y2 = rawData2(:, 3);              % 3열: acc_y
% acc_z2 = rawData2(:, 4);              % 4열: acc_z
% gyr_x2 = rawData2(:, 5);              % 5열: gyr_x
% gyr_y2 = rawData2(:, 6);              % 6열: gyr_y
% gyr_z2 = rawData2(:, 7);              % 7열: gyr_z
% h_d2 = rawData2(:, 8);                % 8열: h_d
% psi_dot_d2 = rawData2(:, 9);          % 9열: psi_dot_d
% psi_dot_hat2 = rawData2(:, 10);       % 10열: psi_dot_hat
% tau_LW2 = rawData2(:, 11);            % 11열: tau_LW
% tau_RW2 = rawData2(:, 12);            % 12열: tau_RW
% theta_dot_hat2 = rawData2(:, 15);     % 15열: theta_dot_hat
% theta_eq2 = rawData2(:, 16);          % 16열: theta_eq
% theta_hat2 = rawData2(:, 17);         % 17열: theta_hat
% v_d2 = rawData2(:, 18);               % 18열: v_d
% v_hat2 = rawData2(:, 19);             % 19열: v_hat
% 
% [f, acc_P1] = get_fft(acc_x1);
% [f, acc_P2] = get_fft(acc_x2);
% acc_x_max = max([acc_P1(2:end); acc_P2(2:end)]);
% [f, acc_P1] = get_fft(acc_y1);
% [f, acc_P2] = get_fft(acc_y2);
% acc_y_max = max([acc_P1(2:end); acc_P2(2:end)]);
% [f, acc_P1] = get_fft(acc_z1);
% [f, acc_P2] = get_fft(acc_z2);
% acc_z_max = max([acc_P1(2:end); acc_P2(2:end)]);
% 
% figure(1);
% subplot(1, 3, 1);
% [f, P1] = get_fft(acc_x1);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_x");
% title("Raw Data FFT");
% ylim([0, acc_x_max]);
% grid on;
% 
% subplot(1, 3, 2);
% [f, P1] = get_fft(acc_y1);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_y");
% title("Raw Data FFT");
% ylim([0, acc_y_max]);
% grid on;
% 
% subplot(1, 3, 3);
% [f, P1] = get_fft(acc_z1);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_z");
% title("Raw Data FFT");
% ylim([0, acc_z_max]);
% grid on;
% 
% figure(2);
% subplot(1, 3, 1);
% [f, P1] = get_fft(acc_x2);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_x");
% title("Raw Data FFT");
% ylim([0, acc_x_max]);
% grid on;
% 
% subplot(1, 3, 2);
% [f, P1] = get_fft(acc_y2);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_y");
% title("Raw Data FFT");
% ylim([0, acc_y_max]);
% grid on;
% 
% subplot(1, 3, 3);
% [f, P1] = get_fft(acc_z2);
% plot(f(2:end), P1(2:end));
% xlabel("frequency domain");
% ylabel("Scaled acc\_z");
% title("Raw Data FFT");
% ylim([0, acc_z_max]);
% grid on;

%
% % 플롯 생성
% figure(1);
%
% % Subplot 1: 가속도 (acc_x, acc_y, acc_z) 비교
% subplot(2, 3, 1);
% plot(timeStamp, acc_x, 'r', 'DisplayName', 'acc\_x'); hold on;
% title('Accelerations');
% xlabel('Time (ms)');
% ylabel('Acceleration (m/s^2)');
% legend('show'); grid on; hold off;
%
% % Subplot 1: 가속도 (acc_x, acc_y, acc_z) 비교
% subplot(2, 3, 2);
% plot(timeStamp, acc_y, 'g', 'DisplayName', 'acc\_y'); hold on;
% title('Accelerations');
% xlabel('Time (ms)');
% ylabel('Acceleration (m/s^2)');
% legend('show'); grid on; hold off;
%
% % Subplot 1: 가속도 (acc_x, acc_y, acc_z) 비교
% subplot(2, 3, 3);
% plot(timeStamp, acc_z, 'b', 'DisplayName', 'acc\_z'); hold on;
% title('Accelerations');
% xlabel('Time (ms)');
% ylabel('Acceleration (m/s^2)');
% legend('show'); grid on; hold off;
%
% % Subplot 2: 자이로스코프 (gyr_x, gyr_y, gyr_z) 비교
% subplot(2, 3, 4);
% plot(timeStamp, gyr_x, 'r', 'DisplayName', 'gyr\_x'); hold on;
% title('Gyroscope');
% xlabel('Time (ms)');
% ylabel('Angular Velocity (rad/s)');
% legend('show'); grid on; hold off;
%
% % Subplot 2: 자이로스코프 (gyr_x, gyr_y, gyr_z) 비교
% subplot(2, 3, 5);
% plot(timeStamp, gyr_x, 'g', 'DisplayName', 'gyr\_y'); hold on;
% title('Gyroscope');
% xlabel('Time (ms)');
% ylabel('Angular Velocity (rad/s)');
% legend('show'); grid on; hold off;
%
% % Subplot 2: 자이로스코프 (gyr_x, gyr_y, gyr_z) 비교
% subplot(2, 3, 6);
% plot(timeStamp, gyr_x, 'b', 'DisplayName', 'gyr\_z'); hold on;
% title('Gyroscope');
% xlabel('Time (ms)');
% ylabel('Angular Velocity (rad/s)');
% legend('show'); grid on; hold off;
%
% % 플롯 생성
% figure(2);
%
% % Subplot 3: psi_dot 비교
% subplot(2, 2, 1);
% plot(timeStamp, psi_dot_d, 'r', 'DisplayName', '\psi\_dot\_d'); hold on;
% plot(timeStamp, psi_dot_hat, 'b', 'DisplayName', '\psi\_dot\_hat');
% title('\psi_{dot}');
% xlabel('Time (ms)');
% ylabel('Value');
% legend('show'); grid on; hold off;
%
% % Subplot 4: tau_LW와 tau_RW 비교
% subplot(2, 2, 2);
% plot(timeStamp, tau_LW, 'r', 'DisplayName', '\tau\_LW'); hold on;
% plot(timeStamp, tau_RW, 'b', 'DisplayName', '\tau\_RW');
% title('\tau_{LW} and \tau_{RW}');
% xlabel('Time (ms)');
% ylabel('Torque');
% legend('show'); grid on; hold off;
%
% % Subplot 5: theta_hat와 theta_eq 비교
% subplot(2, 2, 3);
% plot(timeStamp, theta_hat, 'r', 'DisplayName', '\theta\_hat'); hold on;
% plot(timeStamp, theta_eq, 'b', 'DisplayName', '\theta\_eq');
% title('Theta');
% xlabel('Time (ms)');
% ylabel('Angle (rad)');
% legend('show'); grid on; hold off;
%
% % Subplot 6: 속도 v_d와 v_hat 비교
% subplot(2, 2, 4);
% plot(timeStamp, v_d, 'r', 'DisplayName', 'v\_d'); hold on;
% plot(timeStamp, v_hat, 'b', 'DisplayName', 'v\_hat');
% title('Velocity');
% xlabel('Time (ms)');
% ylabel('Velocity (m/s)');
% legend('show'); grid on; hold off;

