clc 
clear 
close all
%% ��������
RootPath='D:\��Һ������Ϸ���ƽ̨\Һ�޳�Corner�켣����_3D';
%�����־λ·��
java_flag = [RootPath '\java_flag.csv'];
matlab_flag = [RootPath '\matlab_flag.csv'];
%��������ļ�·��
java_output = [RootPath '\java_output.csv'];
matlab_output = [RootPath '\matlab_output.csv'];
%�������Simulinkģ�͵�ַ
model='ALGO_Corner_TrajTrack_MPC_T1_SP_LTR_Cosim_3D';%ʵ�����Ϸ�����ģ��
simulink_model=[model '.mdl'];


%% �����������
SimTime=str2double(get_param(model,'StopTime'));   %���÷���ʱ��
timeStep=0.005; %���÷��沽��
iteration = round(SimTime/timeStep);   %������沽��

MatlabOutVariableNum=6; %Simulink����Ĳ�������  ax ay az ddroll ddpitch ddyaw
StarOutVariableNum=7;   %StarCCM+����Ĳ������� time Fx Fy Fz Mx My Mz
%�����������
a_tank=1.24;
b_tank=0.9;
f_rate=0.5;
LengthOfTank=11;%���ù��峤��
densityOfWater=750;
waterMass=pi*f_rate*a_tank*b_tank*LengthOfTank*densityOfWater;
%% ����StarCCM+�����ʼ״̬
StarInitState=zeros(1,StarOutVariableNum);

StarInitState(3)=waterMass/LengthOfTank*-9.81;%verticalForce


%% ��ʼ������
SimParameters=[iteration,timeStep,MatlabOutVariableNum,StarOutVariableNum];%���������ô��͸�Star
csvwrite(matlab_output,SimParameters, 0, 0);
csvwrite(java_output,StarInitState, 1, 0);
resetFlag(java_flag);
while(~getFlag(java_flag))%�ȴ�Star������������
end
resetFlag(java_flag);
resetFlag(matlab_flag);%��λ��ʾmatlabû׼�������ݣ�Star����ȡ��ʼ����ֵ���е�һ�����棬���ǲ����Զ�����ڶ���

% set_param(model,'StopTime','SimTime');
set_param([model '/StarCCM_Function'],'SystemSampleTime','[timeStep 0]');
tic
SimOut=sim(simulink_model);
runTime=toc;
dispRunTime(runTime);
%% ���������
simfilename=['Corner_' date ' ' datestr(datetime, 'HH-MM-ss')  'MPC_T1_SP_LTR_3D������.mat'];
save(simfilename,'SimOut');




function dispRunTime(runTime)
if(runTime<60)
    disp(['������ɣ���ʱ ' num2str(runTime) 's'])
elseif(runTime<3600)
    disp(['������ɣ���ʱ ' num2str(floor(runTime/60)) 'min ' num2str(mod(runTime,60)) 's'])
else
    disp(['������ɣ���ʱ ' num2str(floor(runTime/3600)) 'h ' num2str(mod(floor(runTime/60),60)) 'min ' num2str(mod(runTime,60)) 's'])
end

end



