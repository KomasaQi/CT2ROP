function acc = fastIDM(params,states)
    % 纵向参数量
    % a_max % 最大加速度
    % b     % 舒适减速度
    % mu   % 路面附着系数
    % v_des % 期望车速
    % delta % 加速度系数
    % T     % 期望跟车时距
    % s_min % 最小车间距（静止时）
    g = 9.806; % 重力加速度

    % 状态量
    % v_ego,v_p,s
    
    dv = v_ego-v_p;
    s_star = s_min + max(0, (v_ego*T + (v_ego*dv/(2*sqrt(a_max*b)))));
    acc = a_max*(1-(v_ego/v_des)^delta - (s_star/s)^2);

    a_cons = g*mu;%obj.g*obj.mu;
    if acc > a_cons
        acc = a_cons;
    elseif acc < -a_cons
        acc = -a_cons;
    end

end

