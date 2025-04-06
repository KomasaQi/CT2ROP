%%  本函数用来对7DOF模型进行一次仿真
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16×1 Nx=16 最后加一个LTR
% 观测量：observe=[Y1,X1,f1,dth1,dth2,LTR]';6×1  Ny=6
function states = simTruck7DOF(tspan,Ts,inputs,params)
    % 定义仿真时间
    time = (tspan(1):Ts:tspan(2))';
    % 定义初始状态
    state = zeros(16,1);
    state(13) = 70/3.6; % 对速度进行赋初值
    % 获取系统动力学微分方程离散表达
    [A,B,C] = TrailorTruck_7DOF_LDP(state, Ts, params);
    states = zeros(length(time),17); % 因为在16个状态最后加了一个LTR
    states(1,:) = [state' 0]; % 将初始值存入
    for i = 2:length(time)
        input = TruckSimInput(inputs(i-1,:)); % 将4个输入变换为6个输入
        state = A*state + B*input;
        observe = C*state;
        states(i,:) = [state' observe(6)]; % 存储结果
    end
    v1y = states(:,1);
    yaw1 = states(:,14);
    v1x = states(:,13);
    % Y1 = cumsum(cos(yaw1).*v1y + sin(yaw1).*v1x)*Ts;
    % X1 = cumsum(-sin(yaw1).*v1y + cos(yaw1).*v1x)*Ts;
    Y1 = cumsum(v1y + yaw1.*v1x)*Ts;
    X1 = cumsum(  v1x)*Ts;
    states(:,[15 16]) = [Y1 X1];
end

%% 子函数：输入单位转换*******************************
function input = TruckSimInput(u)

    delta = u(1);
    M1z   = u(2);
    M2z   = u(3); 
    M1zp  = max(M1z,0);
    M1z_  = max(-M1z,0);
    M2zp  = max(M2z,0);
    M2z_  = max(-M2z,0);
    vdes  = u(4);

    input = [delta, M1zp, M1z_, M2zp, M2z_, vdes]'; %6x1 Nu = 6 
end
