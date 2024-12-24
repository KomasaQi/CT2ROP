% 重新修改的lossfcn用于生成mex加快求解速度
function cost = lossfcn_6order(params,tf,a_,b_,A_d,resolution,L,delta_lim,deltaRate_lim,ltr_zeta,ltr_omega_n) %#codegen
    a6 = params(1); 
    b6 = params(2);
    
    spd_der = (1:6)';
    acc_der = [2 6 12 20 30]';

    a_spd = [a_(2:3);a_(4:6)-A_d*a6;a6].*spd_der;
    b_spd = [b_(2:3);b_(4:6)-A_d*b6;b6].*spd_der;
    
    a_acc = [a_(3);a_(4:6)-A_d*a6;a6].*acc_der;
    b_acc = [b_(3);b_(4:6)-A_d*b6;b6].*acc_der;
    
    time = (0:resolution:tf)';
    % 计算|| LTR ||∞
    dx = polyval(a_spd(end:-1:1),time);
    dy = polyval(b_spd(end:-1:1),time);
    ddx = polyval(a_acc(end:-1:1),time);
    ddy = polyval(b_acc(end:-1:1),time);
    
    ay = (dx.*ddy-dy.*ddx)./sqrt(dx.^2+dy.^2);
    kappa = ay./(dx.^2+dy.^2);
    delta = atan(kappa*L)*180/pi;
    deltaRate = gradient(delta)./gradient(time);
    y0 = zeros(2,1); % LTR;dLTR
    % y0 = [-0.5,10];
    [~,y] = ode45(@(t,y)ltr_2order_dyn(t, y, [time;1.5*tf], [ay;0], ltr_zeta, ltr_omega_n),(0:resolution:1.5*tf),y0);
    LTR = y(:,1);
    % ||LTR||∞为优化目标，利用罚函数施加前轮转角、转角速度约束。
    cost = max(abs(LTR))+20*max(max(abs(delta))-delta_lim,0)+20*max(max(abs(deltaRate))-deltaRate_lim,0);
end


% 将LTR近似为侧向加速度ay的二阶惯性环节
function dy = ltr_2order_dyn(t, y, ay_time, ays, zeta, omega_n)
    LTR = y(1);
    dLTR = y(2);
    ay = interp1(ay_time,ays,t);
    dy = zeros(2,1);
    dy(1) = dLTR;
    dy(2) = ay - 2*zeta*omega_n*dLTR - omega_n^2*LTR; 
end