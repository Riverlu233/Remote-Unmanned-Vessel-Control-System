function [out_data, state_struct] = info_processing(in_data, state_struct, param, mode, valid_flag)
    p = [1, 0]; 
    q_bits = param.quant_bits;
    range = param.quant_range; 
    
    levels = 2^q_bits;
    hist = state_struct.recon_hist; 
    
    s_hat = p(1) * hist(1) + p(2) * hist(2);
    
    if strcmp(mode, 'encode')
        %% Encoding
        curr_val = in_data;
        e = curr_val - s_hat;
        e_clipped = max(min(e, range), -range);
        idx = floor((e_clipped + range) / (2*range) * (levels - 1));
        idx = max(min(idx, levels-1), 0);
        e_q = (idx / (levels-1)) * (2*range) - range;
        s_tilde = s_hat + e_q;
        state_struct.recon_hist = [s_tilde, hist(1)];
        out_data = bitget(idx, q_bits:-1:1);
        
    elseif strcmp(mode, 'decode')
        %% Decoding
        if nargin < 5, valid_flag = true; end
        
        if valid_flag
            rx_bits = in_data;
            idx = 0;
            for i = 1:q_bits
                idx = idx + rx_bits(i) * 2^(q_bits-i);
            end
            e_q = (idx / (levels-1)) * (2*range) - range;
        else
            % Compensate for Loss
            e_q = 0;
        end
        
        s_tilde = s_hat + e_q;
        
        state_struct.recon_hist = [s_tilde, hist(1)];
        out_data = s_tilde;
    end
end
