function [state_next, integral_next, u_out] = control_system(state, theta_ref, ref_point, integral_prev, param)
    % 状态解构
    x = state(1); y = state(2); theta = state(3); omega = state(4);
    
    %% 1. 路径偏差计算
    dx = x - ref_point(1);
    dy = y - ref_point(2);
    % 横向偏差 (Cross-track error)
    de = -dx * sin(theta_ref) + dy * cos(theta_ref);
    
    %% 2. LOS 诱导律
    % 更新积分
    integral_next = integral_prev + de * param.dt;
    % 适当放宽积分限幅，确保有足够的力抵消洋流
    int_limit = 50; 
    integral_next = max(min(integral_next, int_limit), -int_limit);
    
    % 计算诱导角 beta
    % Delta 是前视距离，如果 Delta 太大，修正会很肉；太小会震荡
    beta = atan(-de / param.Delta);
    theta_los = theta_ref + beta - param.Ki_los * integral_next;
    
    %% 3. PD 控制器
    % 必须使用 atan2 确保误差在 [-pi, pi]
    angle_err = atan2(sin(theta - theta_los), cos(theta - theta_los));
    
    % 计算控制力矩
    % 增加一个最小启动力矩补偿，防止“开不出去”
    u_raw = -param.Kp * angle_err - param.Kd * omega;
    
    % 放宽输出限幅 (之前可能限得太死，1500的惯性需要较大的力矩)
    u_out = max(min(u_raw, 100000), -100000); 
    
    %% 4. 动力学与运动学更新
    % 外部环境
    w_J = randn * param.wave_std; 
    
    % 角加速度: d_omega = (Tau - d*omega + noise) / J
    domega = (u_out - param.d * omega + w_J) / param.J;
    
    % 更新状态
    omega_new = omega + domega * param.dt;
    theta_new = theta + omega_new * param.dt;
    
    % 位置更新 (确保 V 足够大能冲出洋流)
    vx = param.V * cos(theta_new) + param.current(1);
    vy = param.V * sin(theta_new) + param.current(2);
    
    x_new = x + vx * param.dt;
    y_new = y + vy * param.dt;
    
    % 结果封装
    state_next = [x_new, y_new, atan2(sin(theta_new), cos(theta_new)), omega_new];
end