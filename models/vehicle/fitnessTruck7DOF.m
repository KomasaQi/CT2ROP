%% 子函数库：拟合精度计算
% *****************半挂式液罐车线性模型*******************************
% 模型： 5DOF半挂车+LDP线性双摆=7DOF模型
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16×1 Nx=16
% 观测量：observe=[Y1,X1,f1,dth1,dth2,LTR]';6×1  Ny=6

function fitness = fitnessTruck7DOF(params,tspan,Ts,inputs,refState_uni,Dataminmax)
    % 进行一次仿真获得线性7DOF模型计算的车液模型状态
    states = simTruck7DOF(tspan,Ts,inputs,params);
    % 将其进行预处理归一化
    DataLen=size(refState_uni,1);
    state_uni=(states-repmat(Dataminmax(1,:),DataLen,1))./repmat(diff(Dataminmax),DataLen,1);

    %裁剪数据
    state_nums = (1:17);
    ref=refState_uni(:,state_nums);
    s_uni=state_uni(:,state_nums);
    fitness=mean(sum((ref-s_uni).^2).*[1 1 10 1  1 1 10 1  1 1 1 1  1  20 20 1  20]);
end