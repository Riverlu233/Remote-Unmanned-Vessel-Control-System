%% main_simulation.m (修正版)
clear; clc; close all;

%% 1. 全局参数初始化 (引入洋流优化版)
sys_param.V = 15.0;              
sys_param.J = 1500;             
sys_param.d = 120;              
sys_param.dt = 0.05;            
sys_param.current = [1.5, -1.0]; % 模拟 X 方向向东, Y 方向向南的洋流
sys_param.wave_std = 10;        

% 控制器增强
sys_param.Kp = 12000;            % 增强转向力度
sys_param.Kd = 5000;            
sys_param.Delta = 10;           % 减小 Delta，提高循迹精度
sys_param.Ki_los = 0.08;         % 显著增大积分项，用于对抗洋流产生的侧偏

% 通信保持稳定
comm_param.SNR_dB = 25;         
comm_param.quant_bits = 8;      
comm_param.quant_range = 15.0;  % 提高量化范围，防止漂移过快导致 DPCM 失锁
comm_param.pred_coeffs = [1, 0];

% 路径说明：0->45->90->135 (左转半圈) -> 回到 0 (斜穿中心) -> -45->-90->-135 (右转半圈)
% 语义参考：0:E(0°), 2:W(180°), 3:N(90°), 1:S(270°/ -90°)
% 5:NE(45°), 7:NW(135°), 6:SW(225°), 4:SE(315°/ -45°)

cmd_schedule = [ 
    0,   0;   % 0-20s:  向东(E)直线航行
    20,  2;   % 20s:    【掉头回头】直接切换为向西(W)，产生180°阶跃，此时船会做大半径掉头
    40,  4;   % 40s:    转向东南(SE)，斜向切回并穿过原有的 E-W 航线（产生交点）
    60,  7;   % 60s:    转向西北(NW)，再次反向斜穿航线（产生第二个交点）
    80,  0];  % 80s:    【再次回头】切回向东(E)
%% 2. 状态初始化
T_total = 100;                  
steps = floor(T_total / sys_param.dt);
state_real = [0, 0, 0, 0]; 

% DPCM 状态重置
decoder_state_x.recon_hist = [0, 0]; 
decoder_state_y.recon_hist = [0, 0];
encoder_state_x.recon_hist = [0, 0];
encoder_state_y.recon_hist = [0, 0];

integral_ye = 0;
current_ref_point = [0, 0]; 
current_cmd_angle = 0;      

log.pos_real = zeros(steps, 2);
log.pos_recon = zeros(steps, 2);
log.cmd_exec = zeros(steps, 1);
log.time = (0:steps-1)*sys_param.dt;

fprintf('开始仿真... \n');

%% 3. 仿真主循环
for k = 1:steps
    t = log.time(k);
    
    % A. 指令处理 (简化逻辑)
    idx = find(cmd_schedule(:,1) <= t, 1, 'last');
    curr_cmd_semantic = cmd_schedule(idx, 2);
    
    % 指令通信
    tx_bits_cmd = bitget(curr_cmd_semantic, 3:-1:1);
    [rx_bits_cmd, cmd_valid_flag] = comm_unit(tx_bits_cmd, comm_param, 'uplink');
    
    if cmd_valid_flag
        rx_cmd_val = rx_bits_cmd(1)*4 + rx_bits_cmd(2)*2 + rx_bits_cmd(3);
        % 语义映射
        angles = [0, -pi/2, pi, pi/2, -pi/4, pi/4, -3*pi/4, 3*pi/4];
        target_angle = angles(rx_cmd_val + 1);
        
        % 航线切换：当角度变化时更新参考起点
        if (target_angle ~= current_cmd_angle) || (k == 1)
            current_ref_point = state_real(1:2);
            integral_ye = 0; 
            current_cmd_angle = target_angle;
        end
    end
    
    % B. 控制与物理演化
    [state_next, integral_ye, ~] = control_system(...
        state_real, current_cmd_angle, current_ref_point, integral_ye, sys_param);
    
    state_real = state_next; % 物理状态更新
    
    % C. 反馈链路 (DPCM)
    [tx_bits_x, encoder_state_x] = info_processing(state_real(1), encoder_state_x, comm_param, 'encode');
    [rx_bits_x, x_v] = comm_unit(tx_bits_x, comm_param, 'downlink');
    [recon_x, decoder_state_x] = info_processing(rx_bits_x, decoder_state_x, comm_param, 'decode', x_v);
    
    [tx_bits_y, encoder_state_y] = info_processing(state_real(2), encoder_state_y, comm_param, 'encode');
    [rx_bits_y, y_v] = comm_unit(tx_bits_y, comm_param, 'downlink');
    [recon_y, decoder_state_y] = info_processing(rx_bits_y, decoder_state_y, comm_param, 'decode', y_v);
    
    % D. 记录
    log.pos_real(k,:) = state_real(1:2);
    log.pos_recon(k,:) = [recon_x, recon_y];
    log.cmd_exec(k) = current_cmd_angle;
end

%% 4. 绘图
figure('Color', 'w', 'Position', [100 100 1000 450]);
subplot(1,2,1);
plot(log.pos_real(:,1), log.pos_real(:,2), 'b-', 'LineWidth', 2); hold on;
plot(log.pos_recon(:,1), log.pos_recon(:,2), 'r--', 'LineWidth', 1);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('无人船航迹图');
legend('真实轨迹', 'DPCM重构轨迹');

subplot(1,2,2);
plot(log.time, rad2deg(log.cmd_exec), 'k', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('指令角度 (deg)');
title('控制指令流');
grid on;