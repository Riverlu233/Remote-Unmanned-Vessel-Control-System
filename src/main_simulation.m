clear; clc; close all;

sys_param.V = 15.0;              
sys_param.J = 1500;             
sys_param.d = 120;              
sys_param.dt = 0.05;            
sys_param.current = [1.5, -1.0];
sys_param.wave_std = 10;        

sys_param.Kp = 12000;
sys_param.Kd = 5000;            
sys_param.Delta = 10;
sys_param.Ki_los = 0.08;

comm_param.SNR_dB = 25;         
comm_param.quant_bits = 8;      
comm_param.quant_range = 15.0;
comm_param.pred_coeffs = [1, 0];

cmd_schedule = [ 
    0,   0;   % 0-20s:  向东(E)直线航行
    20,  2;   % 20s:    直接切换为向西(W)，产生180°掉头
    40,  4;   % 40s:    转向东南(SE)，斜向切回并穿过原有的 E-W 航线
    60,  7;   % 60s:    转向西北(NW)，再次反向斜穿航线
    80,  0];  % 80s:    再次回头,切回向东(E)

T_total = 100;                  
steps = floor(T_total / sys_param.dt);
state_real = [0, 0, 0, 0]; 

% DPCM Reset
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

%% Main Loop
for k = 1:steps
    t = log.time(k);

    idx = find(cmd_schedule(:,1) <= t, 1, 'last');
    curr_cmd_semantic = cmd_schedule(idx, 2);
    
    tx_bits_cmd = bitget(curr_cmd_semantic, 3:-1:1);
    [rx_bits_cmd, cmd_valid_flag] = comm_unit(tx_bits_cmd, comm_param, 'uplink');
    
    if cmd_valid_flag
        rx_cmd_val = rx_bits_cmd(1)*4 + rx_bits_cmd(2)*2 + rx_bits_cmd(3);
        angles = [0, -pi/2, pi, pi/2, -pi/4, pi/4, -3*pi/4, 3*pi/4];
        target_angle = angles(rx_cmd_val + 1);
        
        if (target_angle ~= current_cmd_angle) || (k == 1)
            current_ref_point = state_real(1:2);
            integral_ye = 0; 
            current_cmd_angle = target_angle;
        end
    end
    
    [state_next, integral_ye, ~] = control_system(...
        state_real, current_cmd_angle, current_ref_point, integral_ye, sys_param);
    
    state_real = state_next; % 物理状态更新
    
    % DPCM
    [tx_bits_x, encoder_state_x] = info_processing(state_real(1), encoder_state_x, comm_param, 'encode');
    [rx_bits_x, x_v] = comm_unit(tx_bits_x, comm_param, 'downlink');
    [recon_x, decoder_state_x] = info_processing(rx_bits_x, decoder_state_x, comm_param, 'decode', x_v);
    
    [tx_bits_y, encoder_state_y] = info_processing(state_real(2), encoder_state_y, comm_param, 'encode');
    [rx_bits_y, y_v] = comm_unit(tx_bits_y, comm_param, 'downlink');
    [recon_y, decoder_state_y] = info_processing(rx_bits_y, decoder_state_y, comm_param, 'decode', y_v);
    
    log.pos_real(k,:) = state_real(1:2);
    log.pos_recon(k,:) = [recon_x, recon_y];
    log.cmd_exec(k) = current_cmd_angle;
end

figure('Color', 'w', 'Position', [100 100 1000 450]);
subplot(1,2,1);
plot(log.pos_real(:,1), log.pos_real(:,2), 'b-', 'LineWidth', 2); hold on;
plot(log.pos_recon(:,1), log.pos_recon(:,2), 'r--', 'LineWidth', 1);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('航迹图');
legend('真实轨迹', 'DPCM重构轨迹');

subplot(1,2,2);
plot(log.time, rad2deg(log.cmd_exec), 'k', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('指令角度 (deg)');
title('控制指令流');
grid on;
