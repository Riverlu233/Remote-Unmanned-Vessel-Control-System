%% main_simulation.m (最终集成版：语音数据集随机采样与语义验证)
clear; clc; close all;

%% 1. 系统参数配置
sys_param.V = 15.0; sys_param.J = 1500; sys_param.d = 120;
sys_param.dt = 0.05; sys_param.current = [1.5, -1.0]; sys_param.wave_std = 10;
sys_param.Kp = 12000; sys_param.Kd = 5000; sys_param.Delta = 10; sys_param.Ki_los = 0.08;

% 通信参数 (3bit 语义传输)
comm_param.SNR_dB = 25; comm_param.quant_bits = 8; 
comm_param.quant_range = 15.0; comm_param.pred_coeffs = [1, 0];

% 指令计划 (0:E, 2:W, 4:SE, 7:NW)
cmd_schedule = [0, 0; 25, 2; 50, 4; 75, 7];
% 数据集目录映射 (需与 prepare_voice_data.m 一致)
dir_map = {'one', 'eight', 'down', 'left', 'go', 'right', 'stop', 'yes'}; 
semantic_names = {'正东(E)', '正南(S)', '正西(W)', '正北(N)', '东南(SE)', '东北(NE)', '西南(SW)', '西北(NW)'};

load('voice_train_set.mat');
% 整合到函数需要的结构体中
template_library.features_norm = train_features_norm;
template_library.labels = train_labels;
template_library.mu = mu;
template_library.sigma = sigma;
template_library.p_order = p_order;

%% 3. 仿真初始化
T_total = 100; steps = floor(T_total / sys_param.dt);
state_real = [0, 0, 0, 0]; 
encoder_state_x.recon_hist = [0, 0]; encoder_state_y.recon_hist = [0, 0];
decoder_state_x.recon_hist = [0, 0]; decoder_state_y.recon_hist = [0, 0];

integral_ye = 0; current_cmd_angle = 0; last_idx = -1;
curr_cmd_semantic = 0; % 初始指令

% 日志记录
log.pos_real = zeros(steps, 2);
log.cmd_plan = zeros(steps, 1); % 计划指令
log.cmd_real = zeros(steps, 1); % 实际识别执行指令
log.time = (0:steps-1)*sys_param.dt;

fprintf('--- 仿真开始：开始从数据集随机采样测试 ---\n');

%% 4. 仿真主循环
for k = 1:steps
    t = log.time(k);
    idx = find(cmd_schedule(:,1) <= t, 1, 'last');
    plan_id = cmd_schedule(idx, 2);
    
    % --- A. 施控者语音识别逻辑 (仅在指令切换时触发采样) ---
    if idx ~= last_idx
        % 1. 确定采样文件夹
        target_dir = fullfile('speech_dataset', dir_map{plan_id + 1});
        wav_files = dir(fullfile(target_dir, '*.wav'));
        
        % 2. 随机抽取一个语音样本 s(n)
        rand_idx = randi(length(wav_files));
        [s_raw, fs] = audioread(fullfile(target_dir, wav_files(rand_idx).name));
        
        % 3. 调用语音处理单元 (LPC + 加权 KNN)
        [recognized_id, conf] = voice_processing_unit(s_raw, fs, template_library);
        
        % 4. 更新当前执行指令
        curr_cmd_semantic = recognized_id;
        
        % 5. 控制台输出实时对比
        fprintf('[t=%.2fs] 任务计划: %s | 语音识别结果: %s (置信度: %.2f)\n', ...
            t, semantic_names{plan_id + 1}, semantic_names{recognized_id + 1}, conf);
        if plan_id ~= recognized_id
            fprintf('  >> 注意：检测到语义识别失真(D=1)，无人船将按照误判指令航行！\n');
        end
        last_idx = idx;
    end
    
    % --- B. 指令无线传输 (3bit) ---
    tx_bits = bitget(curr_cmd_semantic, 3:-1:1);
    [rx_bits, ~] = comm_unit(tx_bits, comm_param, 'uplink');
    rx_id = rx_bits(1)*4 + rx_bits(2)*2 + rx_bits(3);
    
    % 映射至物理舵角
    angles = [0, -pi/2, pi, pi/2, -pi/4, pi/4, -3*pi/4, 3*pi/4];
    target_angle = angles(rx_id + 1);
    
    if (target_angle ~= current_cmd_angle) || (k == 1)
        current_ref_point = state_real(1:2);
        integral_ye = 0; current_cmd_angle = target_angle;
    end
    
    % --- C. 控制系统与物理模型 ---
    [state_next, integral_ye, ~] = control_system(...
        state_real, current_cmd_angle, current_ref_point, integral_ye, sys_param);
    state_real = state_next;
    
    % --- D. 反馈链路 (DPCM) ---
    [tx_x, encoder_state_x] = info_processing(state_real(1), encoder_state_x, comm_param, 'encode');
    [rx_x, xv] = comm_unit(tx_x, comm_param, 'downlink');
    [recon_x, decoder_state_x] = info_processing(rx_x, decoder_state_x, comm_param, 'decode', xv);
    
    [tx_y, encoder_state_y] = info_processing(state_real(2), encoder_state_y, comm_param, 'encode');
    [rx_y, yv] = comm_unit(tx_y, comm_param, 'downlink');
    [recon_y, decoder_state_y] = info_processing(rx_y, decoder_state_y, comm_param, 'decode', yv);
    
    % 记录
    log.pos_real(k,:) = state_real(1:2);
    log.cmd_plan(k) = plan_id;
    log.cmd_real(k) = curr_cmd_semantic;
end

%% 5. 结果分析绘图
figure('Color', 'w', 'Position', [100 100 1100 500]);

% 轨迹对比图
subplot(1,2,1);
plot(log.pos_real(:,1), log.pos_real(:,2), 'b-', 'LineWidth', 2); hold on;
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('语音驱动下的无人船航迹');

% 指令执行对比图
subplot(1,2,2);
plot(log.time, log.cmd_plan, 'k--', 'LineWidth', 1.2); hold on;
plot(log.time, log.cmd_real, 'r-', 'LineWidth', 1.5);
ylim([-1, 8]); yticks(0:7);
yticklabels(semantic_names);
legend('计划指令', '语音识别实际指令');
xlabel('时间 (s)'); ylabel('语义指令');
title('计划与实际执行指令对比 (率失真验证)');
grid on;

fprintf('--- 仿真完成 ---\n');
