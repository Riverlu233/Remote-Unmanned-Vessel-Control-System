function [out_data, state_struct] = info_processing(in_data, state_struct, param, mode, valid_flag)
    % INFO_PROCESSING (修正版: 增强稳定性)
    
    % 核心修正：使用一阶预测 [1, 0] 代替不稳定的二阶预测 [2, -1]
    % 如果必须用二阶，建议设置非常大的量化范围或极小的步长
    p = [1, 0]; 
    q_bits = param.quant_bits;
    range = param.quant_range; % 建议在主程序设为 10.0 以上
    
    levels = 2^q_bits;
    hist = state_struct.recon_hist; 
    
    % 1. 计算预测值
    s_hat = p(1) * hist(1) + p(2) * hist(2);
    
    if strcmp(mode, 'encode')
        %% --- 编码过程 ---
        curr_val = in_data;
        e = curr_val - s_hat;
        
        % 限幅
        e_clipped = max(min(e, range), -range);
        
        % 量化索引计算
        idx = floor((e_clipped + range) / (2*range) * (levels - 1));
        idx = max(min(idx, levels-1), 0);
        
        % 逆量化用于本地重构 (确保编解码器同步)
        e_q = (idx / (levels-1)) * (2*range) - range;
        s_tilde = s_hat + e_q;
        
        % 更新历史状态
        state_struct.recon_hist = [s_tilde, hist(1)];
        
        % 输出二进制
        out_data = bitget(idx, q_bits:-1:1);
        
    elseif strcmp(mode, 'decode')
        %% --- 解码过程 ---
        if nargin < 5, valid_flag = true; end
        
        if valid_flag
            rx_bits = in_data;
            idx = 0;
            for i = 1:q_bits
                idx = idx + rx_bits(i) * 2^(q_bits-i);
            end
            e_q = (idx / (levels-1)) * (2*range) - range;
        else
            % 丢包补偿：使用上一次的趋势（残差设为0）
            e_q = 0;
        end
        
        % 重构
        s_tilde = s_hat + e_q;
        
        % 更新历史状态
        state_struct.recon_hist = [s_tilde, hist(1)];
        out_data = s_tilde;
    end
end