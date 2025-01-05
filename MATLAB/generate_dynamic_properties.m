clc;clear;close all;
addpath("lib\");
format long;

% Mainbody
mass_of_Mainbody = 1414.04421338;  % 질량 (g)
center_of_mass_MainBody = [18.19659659; -0.24888409; 35.42269696];  % 무게 중심 위치 (mm)
inertia_tensor_MainBody = [4191453.72496564, 19611.02661734, 183001.27422549;
                           19611.02661734, 6619178.75424983, 6399.95018693;
                           183001.27422549, 6399.95018693, 7735669.49472275];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Mainbody Topcover 제외, 추가 질량 IMU로 변경
% mass_of_Mainbody = 1351.81368770;% 질량 (g)
% center_of_mass_MainBody = [18.96647951; -0.23789954; 31.48638700]; % 무게 중심 위치 (mm)
% inertia_tensor_MainBody = [ 3672853.09619279, -2475.57113883, 4410.32591279; ...
%                             -2475.57113883, 5942293.36188387, -2475.57113883; ...
%                             4410.32591279, -2475.57113883, 7013336.44015658]; % 무게 중심 기준 관성 모멘트 (g mm^2)


% Calf Link Left
mass_of_Calf_Link_Left = 338.23782393;  % 질량 (g)
center_of_mass_Calf_Link_Left = [169.71073180; -3.92577446; 7.13663078];  % 무게 중심 위치 (mm)
inertia_tensor_Calf_Link_Left = [108187.53374990, -5688.74489609, -34349.39868930;
                                  -5688.74489609, 849624.61514512, -272.20346797;
                                  -34349.39868930, -272.20346797, 820318.28517488];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Calf Link Right
mass_of_Calf_Link_Right = 338.23782393;  % 질량 (g)
center_of_mass_Calf_Link_Right = [169.71091698; 3.92574726; 7.13537305];  % 무게 중심 위치 (mm)
inertia_tensor_Calf_Link_Right = [108187.86830599, 5689.13762061, -34348.43584610;
                                   5689.13762061, 849621.31645905, 266.26892472;
                                   -34348.43584610, 266.26892472, 820314.86080829];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Thigh Link Active Left
mass_of_ThighLink_Active_Left = 42.41994494;  % 질량 (g)
center_of_mass_ThighLink_Active_Left = [-48.46903323; 4.61143087; 2.04282631];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Active_Left = [5006.89568398, 6444.24555368, -846.35023128;
                                         6444.24555368, 48849.83835373, 404.38523501;
                                         -846.35023128, 404.38523501, 49929.39396428];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Thigh Link Active Right
mass_of_ThighLink_Active_Right = 42.41994494;  % 질량 (g)
center_of_mass_ThighLink_Active_Right = [-48.46902377; -4.61143086; 2.04281842];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Active_Right = [5006.89699630, -6444.24742526, -846.35096499;
                                          -6444.24742526, 48849.85593152, -404.38374370;
                                          -846.35096499, -404.38374370, 49929.41023364];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Thigh Link Passive Left
mass_of_ThighLink_Passive_Left = 39.26139565;  % 질량 (g)
center_of_mass_ThighLink_Passive_Left = [-77.21916722; 10.27201342; -3.86042407];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Passive_Left = [5253.53554832, 5957.24686321, 2247.31583435;
                                          5957.24686321, 60185.49451452, -799.05607096;
                                          2247.31583435, -799.05607096, 60398.22510949];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Thigh Link Passive Right
mass_of_ThighLink_Passive_Right = 39.26139565;  % 질량 (g)
center_of_mass_ThighLink_Passive_Right = [-77.21916722; -10.27201342; -3.86042407];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Passive_Right = [5253.53554222, -5957.24686336, 2247.31583389;
                                           -5957.24686336, 60185.49451538, 799.05607092;
                                           2247.31583389, 799.05607092, 60398.22511646];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Wheel
mass_of_Wheel = 237.11770281;  % 질량 (g)
center_of_mass_Wheel = [-0.00000687; 0.43740164; -0.00000028];  % 무게 중심 위치 (mm)
inertia_tensor_Wheel = [377169.50236306, -0.00684531, -0.00114686;
                         -0.00684531, 723999.02807526, 0.00006205;
                         -0.00114686, 0.00006205, 377169.52206100];  % 무게 중심 기준 관성 모멘트 (g mm^2)

% Wheel Right
mass_of_Wheel_Right = 214.11770281;  % 질량 (g)
center_of_mass_Wheel_Right = [-0.00000761; 0.48438625; -0.00000031];  % 무게 중심 위치 (mm)
inertia_tensor_Wheel_Right = [334104.60295742, -0.00692262, -0.00045120;
                               -0.00692262, 640742.79507983, 0.00005987;
                               -0.00045120, 0.00005987, 334104.65585037];  % 무게 중심 기준 관성 모멘트 (g mm^2)


% link_parameter
properties.a = 75 * cos(pi/6) * 1e-3;   % (m)
properties.b = 75 * sin(pi/6) * 1e-3;   % (m)
properties.l1 = 106 * 1e-3;             % (m)
properties.l2 = 77 * 1e-3;              % (m)
properties.l3 = 50 * 1e-3;              % (m)
properties.l4 = 137 * 1e-3;             % (m)
properties.l5 = 8 * 1e-3;               % (m)


properties.L = 123 * 1e-3;              % Distance between center and wheel (m)
properties.R = 76.2 * 1e-3;             % Wheel Radius (m)


% 각 link들의 CoM Offset (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.c_vectors = [
    center_of_mass_MainBody, ...
    center_of_mass_ThighLink_Active_Right, ...
    center_of_mass_ThighLink_Active_Left, ...
    center_of_mass_ThighLink_Passive_Right, ...
    center_of_mass_ThighLink_Passive_Left, ...
    center_of_mass_Calf_Link_Right, ...
    center_of_mass_Calf_Link_Left
    ];
properties.c_vectors = properties.c_vectors * 1e-3; % 단위 변환 (mm to m)

% 각 link들의 질량 (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.masses = [
    mass_of_Mainbody, ...
    mass_of_ThighLink_Active_Right, ...
    mass_of_ThighLink_Active_Left, ...
    mass_of_ThighLink_Passive_Right, ...
    mass_of_ThighLink_Passive_Left, ...
    mass_of_Calf_Link_Right, ...
    mass_of_Calf_Link_Left
    ];
properties.masses = properties.masses * 1e-3; % 단위 변환 (g to kg)

% 각 link들의 CoM 기준 관성 모멘트 텐서 (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.IG_matrices(:,:,1) = inertia_tensor_MainBody * 1e-9;
properties.IG_matrices(:,:,2) = inertia_tensor_ThighLink_Active_Right * 1e-9;
properties.IG_matrices(:,:,3) = inertia_tensor_ThighLink_Active_Left * 1e-9;
properties.IG_matrices(:,:,4) = inertia_tensor_ThighLink_Passive_Right * 1e-9;
properties.IG_matrices(:,:,5) = inertia_tensor_ThighLink_Passive_Left * 1e-9;
properties.IG_matrices(:,:,6) = inertia_tensor_Calf_Link_Right * 1e-9;
properties.IG_matrices(:,:,7) = inertia_tensor_Calf_Link_Left * 1e-9;

properties.m_LW = mass_of_Wheel * 1e-3;        % 단위 변환 (g to kg)
properties.m_RW = mass_of_Wheel_Right * 1e-3;  % 단위 변환 (g to kg)

properties.I_B_RW = inertia_tensor_Wheel_Right * 1e-9; % 단위 변환 (g mm^2 to kg m^2)
properties.I_B_LW = inertia_tensor_Wheel * 1e-9;       % 단위 변환 (g mm^2 to kg m^2)

properties.g = 9.80665;    % gravity acceleration (m/s^2)

save('dynamic_properties.mat', "properties");