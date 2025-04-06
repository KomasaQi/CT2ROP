%% *****************���ʽҺ�޳�����ģ�͵�����*******************************
% ģ�ͣ� 5DOF��ҳ�+DLP����˫��=7DOFģ��
% ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16��1 Nx=16
% �۲�����observe=[Y1,X1,f1,dth1,dth2,LTR]';6��1  Ny=6


%% ������ ���ݱ�־λflag���л��������巽��
function [sys,x0,str,ts] = TankTruckLinearModel_DLP(t,x,u,flag)
% �����Խű����ڳ��Ծ���MPC����Ԥ��ģ���е�Ǳ�ڴ���
    %   �޶��ڳ�������ѧģ�ͣ�������Ϊǰ��ƫ��
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
%% ģ���ʼ��
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
    sizes.NumInputs      = 4;  % ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
    %                            th,dth,th2,dth2,f1,vx1,Y1,X1]';16 �� 1, Nx = 16
    %                            + input = [delta, M1zp, M1z_, M2zp, M2z_, vdes]' 
    %                              6 x 1, Nu = 6
    % ��ʵ����ֻ������u = [delta,M1z,M2z,vdes]
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0 =1e-4;   
    global Ts state
    Ts = 0.05;
    state = zeros(16,1);
    % ��״̬�����ֵ
    state(13) = 70/3.6; % ��ʼ�ٶ�
    global c_larger_delta c_smaller_Mz   % ��ǰ����õ�2��ϵ��,����ʹ�����뷶Χ��һ��
    c_larger_delta = 180/pi; % �µ�λ��deg
    c_smaller_Mz = 1e-4; % �µ�λ��10kNm
    % Initialize the discrete states.
    str = [];             % Set str to an empty matrix.
    ts  = [Ts 0];       % sample time: [period, offset]
    %End of mdlInitializeSizes
end		 
%% ������ɢ״̬��
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
    sys = x;
    %End of mdlUpdate.
end
%% ���������
%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global state Ts 
    global c_smaller_Mz c_larger_delta
%% ��ز�������
% ״̬����state = [vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                  th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16 �� 1, Nx = 16
% �۲�����observe = [Y1,X1,f1,dth1,dth2,LTR]';6��1  Ny=6
% ��������input = [delta,M1zp,M1z_,M2zp,M2z_,vdes]'; 6 x 1 Nu=6
    % ת����������λ������Բ�����Ҫת���ģ�
    input = TruckSimInput(u); 
    input(1) = input(1)*c_larger_delta; % ���е�λԤ����
    input(2:5) = input(2:5)*c_smaller_Mz; % ���е�λԤ����

    % ��ȡLTV��Һ��ϼ򻯶���ѧģ��
    [A,B,C]=TrailorTruck_7DOF_LDP(state,Ts);
    
    % % ״̬ת����۲�
    state = A*state + B*input; % B = 16 x 6 input = 6 x 1
    observe = C*state;
    LTR = observe(6);

    % ���״̬��۲���
    sys=[state;LTR]; % steering
% End of mdlOutputs.
end


