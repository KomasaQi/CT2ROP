clc 
clear 
close all
%% 仿真设置
RootPath='D:\车液耦合联合仿真平台\液罐车Corner轨迹跟踪_3D';
%定义标志位路径
java_flag = [RootPath '\java_flag.csv'];
matlab_flag = [RootPath '\matlab_flag.csv'];
%定义输出文件路径
java_output = [RootPath '\java_output.csv'];
matlab_output = [RootPath '\matlab_output.csv'];
%定义仿真Simulink模型地址
model='ALGO_Corner_TrajTrack_MPC_T1_SP_LTR_Cosim_3D';%实际联合仿真用模型
simulink_model=[model '.mdl'];


%% 仿真参数设置
SimTime=str2double(get_param(model,'StopTime'));   %设置仿真时间
timeStep=0.005; %设置仿真步长
iteration = round(SimTime/timeStep);   %求出仿真步数

MatlabOutVariableNum=6; %Simulink输出的参数个数  ax ay az ddroll ddpitch ddyaw
StarOutVariableNum=7;   %StarCCM+输出的参数个数 time Fx Fy Fz Mx My Mz
%罐体参数设置
a_tank=1.24;
b_tank=0.9;
f_rate=0.5;
LengthOfTank=11;%设置罐体长度
densityOfWater=750;
waterMass=pi*f_rate*a_tank*b_tank*LengthOfTank*densityOfWater;
%% 计算StarCCM+输出初始状态
StarInitState=zeros(1,StarOutVariableNum);

StarInitState(3)=waterMass/LengthOfTank*-9.81;%verticalForce


%% 初始化仿真
SimParameters=[iteration,timeStep,MatlabOutVariableNum,StarOutVariableNum];%将仿真设置传送给Star
csvwrite(matlab_output,SimParameters, 0, 0);
csvwrite(java_output,StarInitState, 1, 0);
resetFlag(java_flag);
while(~getFlag(java_flag))%等待Star接收完仿真参数
end
resetFlag(java_flag);
resetFlag(matlab_flag);%复位表示matlab没准备好数据，Star可以取初始条件值进行第一部仿真，但是不能自动进入第二步

% set_param(model,'StopTime','SimTime');
set_param([model '/StarCCM_Function'],'SystemSampleTime','[timeStep 0]');
tic
SimOut=sim(simulink_model);
runTime=toc;
dispRunTime(runTime);
%% 保存仿真结果
simfilename=['Corner_' date ' ' datestr(datetime, 'HH-MM-ss')  'MPC_T1_SP_LTR_3D仿真结果.mat'];
save(simfilename,'SimOut');




function dispRunTime(runTime)
if(runTime<60)
    disp(['仿真完成，用时 ' num2str(runTime) 's'])
elseif(runTime<3600)
    disp(['仿真完成，用时 ' num2str(floor(runTime/60)) 'min ' num2str(mod(runTime,60)) 's'])
else
    disp(['仿真完成，用时 ' num2str(floor(runTime/3600)) 'h ' num2str(mod(floor(runTime/60),60)) 'min ' num2str(mod(runTime,60)) 's'])
end

end



