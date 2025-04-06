%% *****************半挂式液罐车线性模型调整器*******************************
% 模型： 5DOF半挂车+DLP线性双摆=7DOF模型
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16×1 Nx=16
% 观测量：observe=[Y1,X1,f1,dth1,dth2,LTR]';6×1  Ny=6


%% 主函数 根据标志位flag来切换操作具体方程
function [sys,x0,str,ts] = TankTruckLinearModel_DLP(t,x,u,flag)
% 本测试脚本用于尝试纠正MPC车辆预测模型中的潜在错误
    %   限定于车辆动力学模型，控制量为前轮偏角
    %   [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
    switch flag
     case 0
      [sys,x0,str,ts] = mdlInitializeSizes; % Initialization 
     case 2
      sys = mdlUpdates(t,x,u); % Update discrete states
     case 3
      sys = mdlOutputs(t,x,u); % Calculate outputs
     case {1,4,9} % Unused flags
      sys = []; 
     otherwise
      error(['unhandled flag = ',num2str(flag)]); % Error handling
    end
    % End of dsfunc.
end
%% 模块初始化
%==============================================================
% Initialization
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes

    % Call simsizes for a sizes structure, fill it in, and convert it 
    % to a sizes array.
    
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 1; % this parameter doesn't matter
    sizes.NumOutputs     = 17; % state_7dof + LTR
    sizes.NumInputs      = 4;  % 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
    %                            th,dth,th2,dth2,f1,vx1,Y1,X1]';16 × 1, Nx = 16
    %                            + input = [delta, M1zp, M1z_, M2zp, M2z_, vdes]' 
    %                              6 x 1, Nu = 6
    % 但实际上只输入了u = [delta,M1z,M2z,vdes]
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0 =1e-4;   
    global Ts state
    Ts = 0.05;
    state = zeros(16,1);
    % 对状态赋予初值
    state(13) = 70/3.6; % 初始速度
    global c_larger_delta c_smaller_Mz   % 提前定义好的2个系数,用来使得输入范围归一化
    c_larger_delta = 180/pi; % 新单位：deg
    c_smaller_Mz = 1e-4; % 新单位：10kNm
    % Initialize the discrete states.
    str = [];             % Set str to an empty matrix.
    ts  = [Ts 0];       % sample time: [period, offset]
    %End of mdlInitializeSizes
end		 
%% 更新离散状态量
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
    sys = x;
    %End of mdlUpdate.
end
%% 更新输出量
%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global state Ts 
    global c_smaller_Mz c_larger_delta
%% 相关参数定义
% 状态量：state = [vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                  th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16 × 1, Nx = 16
% 观测量：observe = [Y1,X1,f1,dth1,dth2,LTR]';6×1  Ny=6
% 输入量：input = [delta,M1zp,M1z_,M2zp,M2z_,vdes]'; 6 x 1 Nu=6
    % 转换输入量单位（仅针对部分需要转换的）
    input = TruckSimInput(u); 
    input(1) = input(1)*c_larger_delta; % 进行单位预调整
    input(2:5) = input(2:5)*c_smaller_Mz; % 进行单位预调整

    % 获取LTV车液耦合简化动力学模型
    [A,B,C]=TrailorTruck_7DOF_LDP(state,Ts);
    
    % % 状态转移与观测
    state = A*state + B*input; % B = 16 x 6 input = 6 x 1
    observe = C*state;
    LTR = observe(6);

    % 输出状态与观测量
    sys=[state;LTR]; % steering
% End of mdlOutputs.
end


