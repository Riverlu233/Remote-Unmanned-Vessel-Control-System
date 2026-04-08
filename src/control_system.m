function [state_next, integral_next, u_out] = control_system(state, theta_ref, ref_point, integral_prev, param)

    x = state(1); y = state(2); theta = state(3); omega = state(4);
    
    %% calc error
    dx = x - ref_point(1);
    dy = y - ref_point(2);
    de = -dx * sin(theta_ref) + dy * cos(theta_ref);
    
    %% LOS Law
    integral_next = integral_prev + de * param.dt;
    int_limit = 50; 
    integral_next = max(min(integral_next, int_limit), -int_limit);
    
    %% calc inductive angle
    beta = atan(-de / param.Delta);
    theta_los = theta_ref + beta - param.Ki_los * integral_next;
    
    %% PD controller
    % 必须使用 atan2 确保误差在 [-pi, pi]
    angle_err = atan2(sin(theta - theta_los), cos(theta - theta_los));
    u_raw = -param.Kp * angle_err - param.Kd * omega;
    u_out = max(min(u_raw, 100000), -100000); 
    w_J = randn * param.wave_std; 
    domega = (u_out - param.d * omega + w_J) / param.J;
    omega_new = omega + domega * param.dt;
    theta_new = theta + omega_new * param.dt;
    

    vx = param.V * cos(theta_new) + param.current(1);
    vy = param.V * sin(theta_new) + param.current(2);
    
    x_new = x + vx * param.dt;
    y_new = y + vy * param.dt;

    state_next = [x_new, y_new, atan2(sin(theta_new), cos(theta_new)), omega_new];
end
