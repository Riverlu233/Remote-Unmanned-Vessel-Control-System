function [rx_bits, is_valid] = comm_unit(in_bits, param, type)
    % COMM_UNIT 物理层通信单元 (仅微调物理层数学参数，保持原有逻辑结构)
    
    %% 1. 物理层参数设定
    Rs = 100;           
    fc = 1000;          
    L = 100;            
    fs = Rs * L;        
    T_sample = 1/fs;    
    alpha = 0;          % 保持原有的 sinc 脉冲设置
    
    num_bits = length(in_bits);
    
    %% 2. 发送机
    symbols = 2 * in_bits - 1;
    h_rc = rcosdesign(alpha, 1, L, 'normal');
    h_rc = h_rc / sum(h_rc); 
    
    upsampled_symbols = upsample(symbols, L);
    v_t = conv(upsampled_symbols, h_rc, 'same');
    
    t = (0:length(v_t)-1) * T_sample;
    s_t = v_t .* sin(2*pi*fc*t);
    
    %% 3. AWGN 信道 (微调此处的噪声计算逻辑，确保对齐 Eb/N0)
    snr_linear = 10^(param.SNR_dB/10);
    Ps = mean(s_t.^2);
    % 修正 sigma2 计算： Ps * L / (2 * snr_linear) 是符合相干解调理论的
    sigma2 = (Ps * L) / (2 * snr_linear); 
    noise = sqrt(sigma2) * randn(size(s_t));
    r_t = s_t + noise;
    
    %% 4. 接收机
    p0 = sum(in_bits == 0) / num_bits;
    p1 = sum(in_bits == 1) / num_bits;
    if p0 == 0 || p1 == 0, p0 = 0.5; p1 = 0.5; end 
    
    g_a_ones = upsample(ones(1, num_bits), L);
    g_a = conv(g_a_ones, h_rc, 'same');
    temp_phi = g_a .* sin(2*pi*fc*t);
    
    mid_idx = round(num_bits/2);
    center_sample = (mid_idx - 1) * L + 1;
    range_idx = (center_sample - L/2) : (center_sample + L/2 - 1);
    range_idx = range_idx(range_idx > 0 & range_idx <= length(temp_phi));
    E = sum(temp_phi(range_idx).^2) * T_sample;
    
    phi = (1/sqrt(E)) .* temp_phi;
    
    s_mixed = r_t .* phi;
    y_samples = zeros(1, num_bits);
    for m = 1:num_bits
        center_idx = (m - 1) * L + 1;
        idx_range = (center_idx - L/2) : (center_idx + L/2 - 1);
        idx_range = idx_range(idx_range > 0 & idx_range <= length(s_mixed));
        y_samples(m) = sum(s_mixed(idx_range)) * T_sample;
    end
    
    N0 = 2 * sigma2 * T_sample;
    gamma = (N0 / (4*sqrt(E))) * log(p0/p1);
    
    rx_raw_bits = y_samples > gamma;
    
    %% 5. 协议层逻辑 (完全保留，不作改动)
    if strcmp(type, 'uplink')
        if length(rx_raw_bits) == 9
            g1 = rx_raw_bits(1:3);
            g2 = rx_raw_bits(4:6);
            g3 = rx_raw_bits(7:9);
            val1 = bi2de(g1, 'left-msb');
            val2 = bi2de(g2, 'left-msb');
            val3 = bi2de(g3, 'left-msb');
            
            if val1 == val2, rx_bits = g1; is_valid = true;
            elseif val1 == val3, rx_bits = g1; is_valid = true;
            elseif val2 == val3, rx_bits = g2; is_valid = true;
            else, rx_bits = g1; is_valid = false; end
        else
            rx_bits = rx_raw_bits; is_valid = true;
        end
    else
        rx_bits = rx_raw_bits; is_valid = true;
    end
end

function val = bi2de(bits, order)
    % 简易二进制转十进制 (完全保留)
    val = 0;
    n = length(bits);
    for i = 1:n
        val = val + bits(i) * 2^(n-i);
    end
end