
clc;clear;close all;
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');
properties.IG_matrices()

h = 0.2; % (m)
phi = 0;

[mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties);
m_B = mass_total;
p_bcom = r_total;
I_B_B = I_total
L = properties.L;
R = properties.R;
m_RW = mass_of_Wheel_Right * 1e-3;
m_LW = mass_of_Wheel * 1e-3;

theta_eq = atan(-r_total(1) / (h + r_total(3)));
% disp('theta_eq(deg) : ');
% disp(theta_eq * 180 / pi);

B = B_f(L, R);

M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),theta_eq);

nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
    L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), 0,theta_eq,0,0);

% disp('Minv_B : ');
% disp('tau_RW                tau_LW');
% disp(M\B);
% disp('-Minv_nle : ');
% disp(-M\nle);

epsilon = 1e-9;

dM_dtheta = dM_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3),theta_eq);

dnle_dtheta = dnle_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
    0,theta_eq,0,0);

dnle_dqdot = dnle_dqdot_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
    0,theta_eq,0,0);

A_ = calculate_fx(M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot);

B_ = calculate_fu(M, B);

% 컨트롤 가능성 행렬 계산
C = ctrb(A_, B_);  % ctrb 함수는 MATLAB 내장 함수로, 컨트롤 가능성 행렬을 계산

% 랭크 확인
rank_C = rank(C);  % 컨트롤 가능성 행렬의 랭크를 계산


% 샘플링 시간 (T)
Ts = 0.012;
dt = 0.0001; % simulation dt

% 이산 시스템 변환
sys_c = ss(A_, B_, [], []);        % 연속 시스템 생성 (C, D 없음)
sys_d = c2d(sys_c, Ts, 'zoh');    % ZOH 방식으로 이산화
Ad = sys_d.A;                    % 이산화된 A 행렬
Bd = sys_d.B;                    % 이산화된 B 행렬

% 상태 가중치(Q)와 입력 가중치(R) 정의
% Q_ = diag([1 1 1 1]);  % 상태 가중치
% R_ = diag([1 1]);            % 입력 가중치

Q_ = diag([10 1 1000 1000]);  % 상태 가중치
R_ = diag([1e4 1e4]);            % 입력 가중치

% LQR Gain 계산
K_d = dlqr(Ad, Bd, Q_, R_)

% 폐루프 시스템 행렬
Acl = Ad - Bd * K_d;

% 폐루프 극점 계산
closed_loop_poles = eig(Acl);

% 결과 출력
disp('Closed-loop poles:');
disp(closed_loop_poles);


% 극점 시각화
figure;
scatter(real(closed_loop_poles), imag(closed_loop_poles), 'filled');
xlabel('Real Part');
ylabel('Imaginary Part');
title('Closed-Loop Pole Locations (Discrete System)');
grid on;

% 단위원 시각화
hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), '--r', 'LineWidth', 1); % 단위원
axis equal;


% EKF covariance matrix
P = eye(4); % 초기 추정 오차 공분산 행렬
R_cov = diag([1.54239e-3, 1.93287e-3, 2.36791e-3, 3.11351e-6, 4.12642e-6, 5.37135e-6, 1e-5, 1e-5]); % 측정 오차 공분산 행렬
Q_cov = diag([4e-5, 1e-4, 4e-5, 1e-4]); % 프로세스 노이즈 공분산 행렬

% 초기 설정
x_eq = [theta_eq; 0; 0; 0];
x = x_eq + [1 * pi/180; 0; 0; 0];
x_hat = x + [0 * pi/180; 0; 0; 0];
del_x = zeros(4, 1);

x_d = [theta_eq; 0; 1; 0];

xi = zeros(3,1) + [x(1);0;0];

u = [0; 0];
x_log = zeros(4, 1);
x_hat_log = zeros(4,1);
u_log = zeros(2, 1);
xi_log = zeros(3,1);

% 시뮬레이션
idx = 1;
for t = 0:dt:3
    del_x = x - x_eq;
    del_x_d = x_d - x_eq;

    % 시스템 행렬 계산
    M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
        I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),x(1));

    nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
        L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), x(4),x(1),x(2),x(3));


    % 샘플링 시간에 따라 입력 업데이트
    if mod(idx, floor(Ts / dt)) == 0
        M_hat = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),x_hat(1));

        dM_dtheta_hat = dM_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3),x_hat(1));

        nle_hat = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
            L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), x_hat(4),x_hat(1),x_hat(2),x_hat(3));

        dnle_dtheta_hat = dnle_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
            x_hat(4),x_hat(1),x_hat(2),x_hat(3));

        dnle_dqdot_hat = dnle_dqdot_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
            x_hat(4),x_hat(1),x_hat(2),x_hat(3));
        
        % 측정 벡터 (예시)
        % true 값을 통해 sensor 값 추출
        [~, f, ~] = predict_state(x_prev, u, M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot, B,  dt);
        [z, ~] = predict_measurement(f, x_prev, g, h, L, R);
        
        acc_noise_std = [0.039273; 0.043964; 0.048661];
        gyro_noise_std = [0.0017645; 0.0020313; 0.0023176];
        % 입력에 노이즈 추가
        acc_noise = acc_noise_std .* randn(3,1);
        gyro_noise = gyro_noise_std .* randn(3,1);
        
        z(1:3) = z(1:3) + acc_noise;
        z(4:6) = z(4:6) + gyro_noise;

        

        % z(1:3) = 

        % 예측 단계 (Prediction)
        [x_pred, dx, F] = predict_state(x_hat, u, M_hat, dM_dtheta_hat, nle_hat, dnle_dtheta_hat, dnle_dqdot_hat, B,  Ts);
        P_pred = F * P * F' + Q_cov; % 예측된 오차 공분산 행렬
        % dx = zeros(4,1);
        % 업데이트 단계 (Update)
        [h_obs, H] = predict_measurement(dx, x_pred, g, h, L, R);
        K = P_pred * H' / (H * P_pred * H' + R_cov); % 칼만 이득
        x_hat = x_pred + K * (z - h_obs); % 상태 업데이트
        P = (eye(length(x_prev)) - K * H) * P_pred; % 오차 공분산 업데이트
        
        del_x_hat = x_hat - x_eq;
        
        % 정의된 범위
        u_min = -0.5;
        u_max = 0.5;

        % u 계산
        u = -K_d * (del_x_hat - del_x_d);

        % Saturation 적용
        u = max(min(u, u_max), u_min);
    end
    noise_std = 0.01;
    noise_mean = 0;
    % 입력에 노이즈 추가
    noise = noise_std * randn(size(u)) + noise_mean;
    % u = u + noise;

    u_real = u + noise;
    if(abs(u_real(1)) < 0.034)
        u_real(1) = 0;
    end
    if(abs(u_real(2)) < 0.034)
        u_real(2) = 0;
    end
    
    % 다음 상태 계산
    x_prev = x;
    x = step(x, M, nle, B, u_real, dt);
    % x = x;

    xi = xi + x(2:4)*dt;


    % 데이터 저장
    x_log(:, idx) = x_prev;
    x_hat_log(:, idx) = x_hat;
    u_log(:, idx) = u_real;
    xi_log(:, idx) = xi;
    idx = idx + 1;
end
% 각도 단위 변환 (rad → deg)
x_log(1:2, :) = x_log(1:2, :) * 180 / pi;
x_log(4, :) = x_log(4, :) * 180 / pi;
x_hat_log(1:2, :) = x_hat_log(1:2, :) * 180 / pi;
x_hat_log(4, :) = x_hat_log(4, :) * 180 / pi;
x_d(1:2) = x_d(1:2) * 180 / pi;
x_d(4) = x_d(4) * 180 / pi;
xi_log(1,:) = xi_log(1,:) * 180 / pi;
xi_log(3,:) = xi_log(3,:) * 180 / pi;



% 시간 벡터 생성
time = 0:dt:(size(x_log, 2) - 1) * dt;

% 상태 변수 플로팅 (\theta)
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1);
plot(time, x_log(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(1, :), 'k:', 'LineWidth', 1.5);
yline(x_d(1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;
title('State Variable: \theta');
legend('x', 'x_{hat}' ,'x_{d}', 'Location', 'best');
ylim([-30, 30]); % Adjust as needed

% 상태 변수 플로팅 (\dot{\theta})
subplot(2,3,2);
plot(time, x_log(2, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(2, :), 'k:', 'LineWidth', 1.5);
yline(x_d(2), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(1,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\theta (deg/s)');
grid on;
title('State Variable: d\theta');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-100, 100]); % Adjust as needed

% 상태 변수 플로팅 (v)
subplot(2,3,4);
plot(time, x_log(3, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(3, :), 'k:', 'LineWidth', 1.5);
yline(x_d(3), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v (m/s)');
grid on;
title('State Variable: v');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-4, 4]); % Adjust as needed

% 상태 변수 플로팅 (\dot{\psi})
subplot(2,3,5);
plot(time, x_log(4, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(4, :), 'k:', 'LineWidth', 1.5);
yline(x_d(4), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(3,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\psi (deg/s)');
grid on;
title('State Variable: d\psi');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-100, 100]); % Adjust as needed

% 입력 변수 플로팅 (u_1)
subplot(2,3,3);
plot(time, u_log(1, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_1');
grid on;
title('Control Input: u_1');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed

% 입력 변수 플로팅 (u_2)
subplot(2,3,6);
plot(time, u_log(2, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_2');
grid on;
title('Control Input: u_2');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed




function x_pred = step(x, M, nle, B, u, dt)
x_pred = zeros(size(x));
dq = x(2:4);
% disp(dq');

dq_next = dq + M \ (-nle + B * u) * dt;
% disp(dq_next');
x_pred(1) = x(1) + dq_next(1)*dt;
x_pred(2:4) = dq_next;
end

function [x_pred, f, F] = predict_state(x, u, M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot, B,  dt)
f = [x(2); M \ (-nle + B * u)];
x_pred = x + f * dt;

M_inv = M \ eye(3,3);
F = zeros(4,4);
F(1,2) = 1;
F(2:4,1) = -M_inv * (dM_dtheta * M_inv * (-nle + B * u) + dnle_dtheta);
F(2:4, 2:4) = -M_inv*dnle_dqdot;
end

function [h_obs, H] = predict_measurement(f, x_pred, g, h, L, R)
% 미리 계산된 sin과 cos
% theta_ddot = f(2);
% v_dot = f(3);
% psi_ddot = f(4);
theta_ddot = 0;
v_dot = 0;
psi_ddot = 0;



theta = x_pred(1);
theta_dot = x_pred(2);
v = x_pred(3);
psi_dot = x_pred(4);


cos_theta = cos(theta);
sin_theta = sin(theta);

% h_obs 계산
h_obs = zeros(8, 1); % 결과를 저장할 벡터
h_obs(1) = h * theta_ddot + v_dot * cos_theta - g * sin_theta - h * psi_dot^2 * cos_theta * sin_theta;
h_obs(2) = psi_dot * v + h * psi_ddot * sin_theta + h * psi_dot * theta_dot * cos_theta * 2.0;
h_obs(3) = -h * theta_dot^2 + g * cos_theta + v_dot * sin_theta - psi_dot^2 * (h - h * cos_theta^2);
h_obs(4) = -psi_dot * sin_theta;
h_obs(5) = theta_dot;
h_obs(6) = psi_dot * cos_theta;
h_obs(7) = theta_dot - v / R - (L * psi_dot) / R;
h_obs(8) = -theta_dot + v / R - (L * psi_dot) / R;

% H 행렬 계산
H = zeros(8, 4); % 8x4 크기의 행렬
H(1, 1) = psi_dot^2 * (h * sin_theta^2 - h * cos_theta^2) - g * cos_theta - v_dot * sin_theta;
H(2, 1) = h * psi_ddot * cos_theta - h * psi_dot * theta_dot * sin_theta * 2.0;
H(3, 1) = v_dot * cos_theta - g * sin_theta - h * psi_dot^2 * cos_theta * sin_theta * 2.0;
H(4, 1) = -psi_dot * cos_theta;
H(5, 1) = 0.0;
H(6, 1) = -psi_dot * sin_theta;
H(7, 1) = 0.0;
H(8, 1) = 0.0;

H(1, 2) = 0.0;
H(2, 2) = h * psi_dot * cos_theta * 2.0;
H(3, 2) = h * theta_dot * -2.0;
H(4, 2) = 0.0;
H(5, 2) = 1.0;
H(6, 2) = 0.0;
H(7, 2) = 1.0;
H(8, 2) = -1.0;

H(1, 3) = 0.0;
H(2, 3) = psi_dot;
H(3, 3) = 0.0;
H(4, 3) = 0.0;
H(5, 3) = 0.0;
H(6, 3) = 0.0;
H(7, 3) = -1.0 / R;
H(8, 3) = 1.0 / R;

H(1, 4) = h * psi_dot * cos_theta * sin_theta * -2.0;
H(2, 4) = v + h * theta_dot * cos_theta * 2.0;
H(3, 4) = psi_dot * (h - h * cos_theta^2) * -2.0;
H(4, 4) = -sin_theta;
H(5, 4) = 0.0;
H(6, 4) = cos_theta;
H(7, 4) = -L / R;
H(8, 4) = -L / R;
end


function fx = calculate_fx(M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot)
fx = zeros(4,4);
fx(1,2) = 1;
M_inv = M \ eye(3,3);
fx(2:4,1) = M_inv * (dM_dtheta * M_inv * nle - dnle_dtheta);
fx(2:4,2:4) = - M_inv * dnle_dqdot;
end

function fu = calculate_fu(M, B)
fu = zeros(4,2);
fu(2:4,:) = M \ B;
end

function p_vectors = calculate_p_vectors(theta_hips, properties)
theta_hips
p_vectors = zeros(3,7);
[~, ~, ~, c_poses] = solve_forward_kinematics(theta_hips, properties);
c_posR = c_poses(:,1);
c_posL = c_poses(:,2);

% Body
p_vectors(:,1) = [0; 0; 0];
% TAR
p_vectors(:,2) = [-64.951905284*1e-3 ; -86*1e-3; 37.5*1e-3];
% TAL
p_vectors(:,3) = [-64.951905284*1e-3 ; 86*1e-3; 37.5*1e-3];
% TPR
p_vectors(:,4) = [0; -81*1e-3; 0];
% TPL
p_vectors(:,5) = [0; 81*1e-3; 0];
% CR
p_vectors(:,6) = [c_posR(1); -102*1e-3; c_posR(2)];
% CL
p_vectors(:,7) = [c_posL(1); 102*1e-3; c_posL(2)];

end

function R_matrices = calculate_R_matrices(theta_hips, properties)
R_matrices = zeros(3,3,7);
[theta_As, theta_Bs, theta_ks, ~] = solve_forward_kinematics(theta_hips, properties);
% Body
R_matrices(:,:,1) = eye(3,3);
% TAR
R_matrices(:,:,2) = rotation_matrix_y(theta_Bs(1));
% TAL
R_matrices(:,:,3) = rotation_matrix_y(theta_Bs(2));
% TPR
R_matrices(:,:,4) = rotation_matrix_y(theta_As(1));
% TPL
R_matrices(:,:,5) = rotation_matrix_y(theta_As(2));
% CR
R_matrices(:,:,6) = rotation_matrix_y(theta_ks(1));
% CL
R_matrices(:,:,7) = rotation_matrix_y(theta_ks(2));
end

function R = rotation_matrix_y(theta)
% 회전 각도 theta에 대한 y-axis 회전 행렬
% theta는 라디안 단위로 입력

R = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
end

function [mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties)
% 주어진 height과 roll angle에 대해서, system의 Inertia tensor를 계산한다. 이때 frame과 중심은 IMU를 기준으로 한다.
theta_hips = solve_inverse_kinematics(h, phi, properties);

% Extract properties
c_vectors = properties.c_vectors;
masses = properties.masses;
IG_matrices = properties.IG_matrices;

mass_total = sum(masses);

p_vectors = calculate_p_vectors(theta_hips, properties);
R_matrices = calculate_R_matrices(theta_hips, properties);
r_total = zeros(3,1);
I_total = zeros(3,3);

% calculate Body_CoM_offset
for i = 1:7
    r_Bi = p_vectors(:,i) + R_matrices(:,:,i) * c_vectors(:,i);
    r_total = r_total + masses(i) * r_Bi;
end
r_total = r_total / mass_total;

% calculate Body_CoM_Inertia_Tensor
for i = 1:7
    p = p_vectors(:,i);
    R = R_matrices(:,:,i);
    c = c_vectors(:,i);
    m = masses(i);

    IG_ii = IG_matrices(:,:,i);
    r_Bi = p + R * c;
    r_CoMi = r_Bi - r_total;

    IG_Bi =  R * IG_ii * R' + m*(norm(r_CoMi)^2*eye(3,3) - r_CoMi * r_CoMi');
    I_total = I_total + IG_Bi;
end
end

function [theta_As, theta_Bs, theta_ks, c_poses, e_poses] = solve_forward_kinematics(theta_hips, properties)
% theta_hips = [theta_hipR; theta_hipL]

% Extract link length
a = properties.a;
b = properties.b;
l1 = properties.l1;
l2 = properties.l2;
l3 = properties.l3;
l4 = properties.l4;
l5 = properties.l5;

% 계산을 위해 두 hip 각도를 사용
theta_Bs = [-theta_hips(1); theta_hips(2)]; % BR, BL 각도

% L1, L2 계산
L1 = sqrt((a + l2 * cos(theta_Bs)).^2 + (b + l2 * sin(theta_Bs)).^2); % 거리 계산
L2 = (a^2 + b^2 + l1^2 + l2^2 - l3^2 + 2*l2.*(a*cos(theta_Bs) + b*sin(theta_Bs))) / (2 * l1);

% Ratio 확인 및 예외 처리
ratio = L2 ./ L1;
if any(abs(ratio) > 1)
    error('Ratio exceeds valid range. Check input parameters or theta_hips.');
end

% 각도 계산
alpha = atan2(b + l2 * sin(theta_Bs), a + l2 * cos(theta_Bs)); % atan2로 정확한 사분면 계산
theta_As = -acos(ratio) + alpha; % 각도 A 계산

% 무릎 각도 계산
theta_ks = atan2((b + l2 * sin(theta_Bs) - l1 * sin(theta_As)), ...
    (a + l2 * cos(theta_Bs) - l1 * cos(theta_As))); % 무릎 각도

% C 위치 계산
c_poses = [-a - l2 * cos(theta_Bs), b + l2 * sin(theta_Bs)]'; % 좌표 계산

e_poses = [-l1 * cos(theta_As) + l4 * cos(theta_ks) + l5 * sin(theta_ks), ...
    l1 * sin(theta_As) - l4 * sin(theta_ks) + l5 * cos(theta_ks)]';

end

function theta_hips = solve_inverse_kinematics(h, phi, properties)
% Extract link length
a = properties.a;
b = properties.b;
l1 = properties.l1;
l2 = properties.l2;
l3 = properties.l3;
l4 = properties.l4;
l5 = properties.l5;

L = properties.L;

% h_saturation 설정
phi_max = min(atan((h - 60*1e-3) / L), atan((200*1e-3 - h) / L));
phi_min = -phi_max;

if phi > phi_max
    phi = phi_max;
elseif phi < phi_min
    phi = phi_min;
end
hR = h - L*tan(phi);
hL = h + L*tan(phi);

hs = [hR; hL];


AB = sqrt(a^2 + b^2);
angle_ADE = acos((l1^2 + l4^2 + l5^2 - hs.^2) / (2 * l1 * sqrt(l4^2 + l5^2)));
angle_EDF = atan(l5 / l4);
angle_ADC = pi - (angle_ADE + angle_EDF);
AC = sqrt(l1^2 + l3^2 - 2 * l1 * l3 * cos(angle_ADC));

S = (AB^2 + l2^2 - AC.^2) / (2 * AB * l2);

% 요소별로 조건 확인
invalid_indices = abs(S) > 1 & abs(AC - sqrt(a^2 + b^2) - l2) < 0.1;
for i = 1:length(S)
    if invalid_indices(i)
        AC(i) = sqrt(a^2 + b^2) + l2;
        S(i) = 1;
    end
end
angle_ABC = real(acos((AB^2 + l2^2 - AC.^2) / (2 * AB * l2)));
theta_hips = (5 * pi / 6) - angle_ABC;

theta_hips(2) = -theta_hips(2);
end



function [x_pred, F] = statePrediction(x, u, dt)
% 비선형 상태 방정식 (예시)
f = @(x, u) [x(2) + dt * x(3);
    -sin(x(1)) + u(1);  % 예시 동역학
    cos(x(1)) * u(2);   % 예시 동역학
    x(4)];

% 상태 예측
x_pred = f(x, u);

% Jacobian 행렬 계산 (선형화)
F = jacobianState(x, u, dt); % Jacobian 행렬을 계산하는 함수
end

% 측정 예측 함수 (비선형 측정 방정식)
function [H, h] = measurementPrediction(x_pred)
% 비선형 측정 방정식 (예시)
h = @(x_pred) [x_pred(1);  % 예시로 측정하는 값
    x_pred(2);  % 예시로 측정하는 값
    x_pred(3);  % 예시로 측정하는 값
    x_pred(4);  % 예시로 측정하는 값
    0;          % 측정값
    0;          % 측정값
    0;          % 측정값
    0];         % 측정값

% 측정 예측값
h = h(x_pred);

% Jacobian 행렬 계산 (측정 방정식 선형화)
H = jacobianMeasurement(x_pred); % 측정 방정식에 대한 Jacobian을 계산하는 함수
end

% 상태 방정식에 대한 Jacobian 계산
function F = jacobianState(x, u, dt)
% 비선형 상태 방정식의 Jacobian 계산 (예시)
F = [1, dt, 0, 0;     % 상태 변수에 대한 편미분
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end

% 측정 방정식에 대한 Jacobian 계산
function H = jacobianMeasurement(x_pred)
% 비선형 측정 방정식의 Jacobian 계산 (예시)
H = [1, 0, 0, 0;     % 상태 변수에 대한 편미분
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
end

