clc;clear;close all;
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');

phi = 0;
% 상태 가중치(Q)와 입력 가중치(R) 정의
Q_ = diag([10 10 5000 100]);  % 상태 가중치
R_ = diag([1e4 1e4]);            % 입력 가중치

% 샘플링 시간 (T)
Ts = 0.012;

Ks = [];
for h = 0.07:0.01:0.2
[mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties);
m_B = mass_total;
p_bcom = r_total;
I_B_B = I_total;
L = properties.L;
R = properties.R;
m_RW = mass_of_Wheel_Right * 1e-3;
m_LW = mass_of_Wheel * 1e-3;

theta_eq = atan(-r_total(1) / (h + r_total(3)));

B = B_f(L, R);

M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),theta_eq);

nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
    L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), 0,theta_eq,0,0);

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

% 이산 시스템 변환
sys_c = ss(A_, B_, [], []);        % 연속 시스템 생성 (C, D 없음)
sys_d = c2d(sys_c, Ts, 'zoh');    % ZOH 방식으로 이산화
Ad = sys_d.A;                    % 이산화된 A 행렬
Bd = sys_d.B;                    % 이산화된 B 행렬

% LQR Gain 계산
K_d = dlqr(Ad, Bd, Q_, R_);
Ks = [Ks; K_d];
end

% 데이터의 크기 확인 (행렬의 행과 열)
[numRows, numCols] = size(Ks);

% C++ 코드 형식으로 출력
for i = 1:numRows/2
    fprintf("mat << ");
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff,\n', Ks(2*i-1,1), Ks(2*i-1,2), Ks(2*i-1,3), Ks(2*i-1,4));
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff;\n', Ks(2*i,1), Ks(2*i,2), Ks(2*i,3), Ks(2*i,4));
    fprintf('Ks.push_back(mat);');
    fprintf('\n\n');
end


function x_next = step(x, M, nle, B, u, dt)
x_next = zeros(size(x));
dq = x(2:4);
% disp(dq');

dq_next = dq + M \ (-nle + B * u) * dt;
% disp(dq_next');
x_next(1) = x(1) + dq_next(1)*dt;
x_next(2:4) = dq_next;
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