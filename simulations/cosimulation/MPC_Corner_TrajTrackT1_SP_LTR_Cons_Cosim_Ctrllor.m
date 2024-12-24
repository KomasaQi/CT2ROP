%% *****************MPC控制器介绍*********************************
% 模型： 5DOF半挂车+SP单摆=6DOF模型
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18×1 Nx=18
% 观测量：observe=[Y1,X1,f1,th,dth,LTR]';6×1  Ny=6
% 控制器：对输出进行约束

%% 主函数 根据标志位flag来切换操作具体方程
% MPC_TrajTrackT1_TPSP_LTR_Ctrllor
function [sys,x0,str,ts] = MPC_Corner_TrajTrackT1_SP_LTR_Cons_Cosim_Ctrllor(t,x,u,flag)
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
sizes.NumDiscStates  = 6; % this parameter doesn't matter
sizes.NumOutputs     = 3; %[steering,LTR]
sizes.NumInputs      = 18;% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                            th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18×1 Nx=18
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =1e-4*ones(6,1);   
global U; % store current ctrl vector: delta_m
U=0;
global path refHead len LTR ctrlMode modeCounter
modeCounter=100;
LTR=0;%初始LTR
ctrlMode=1;%控制模式有两个：1跟踪抑晃 和 2防侧翻
[path,~,len]=getPath('CornerMain');
%获得参考航向角
diff_x=diff(path(:,1));
diff_y=diff(path(:,2));
refHead=zeros(size(path,1),1);
refHead(1:end-1) = atan2(diff_y,diff_x);
refHead(end)=refHead(end-1);
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.05 0];       % sample time: [period, offset]
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
global path len refHead U LTR ctrlMode modeCounter
%% 计算LTR决定控制模式
if abs(LTR)<=0.5   %后面的数字是LTR阈值
    modeCounter=modeCounter+1;
else 
    modeCounter=0;
end
if modeCounter>30 %超过2s LTR加权均值小于0.5
    ctrlMode=1;  %轨迹跟踪+抑晃为主
    fprintf([num2str(t) 's切换模式到轨迹跟踪\n'])
else
    ctrlMode=2;  %防侧翻为主
    fprintf([num2str(t) 's 切换模式到防侧翻\n'])
end
%% 相关参数定义
%MPC控制器参数
% 观测量：observe=[Y1,X1,f1,th,dth,LTR]';6×1  Ny=6
if ctrlMode==1 %轨迹跟踪+抑晃
%     Q=diag([650,650,250,3,4,5]);
    Q=diag([4050,4050,4050,1,1,1]);
    R=200;  
else    % 防侧翻模式
%     Q=diag([950,950,550,2,2,3]);
    Q=diag([4050,4050,4050,1,1,1]);
    R=200;  
end
Np=40;
Nc=3;
rho=180000;   %松弛因子系数
Ts=0.05;   %控制步长，单位：s
%% 主程序
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18×1 Nx=18
    %组合新的状态量：[模型状态量；控制量]’（Nx+Nu）x 1
    state=TruckSimInput(u);
    delta=U;
    pos=[u(16),u(15)];
    % 获得参考量
    [idx,idxend]=findTargetIdx(pos,path,len,state(13),Ts,Np);
    %MPC控制器
    [ddelta,observe]=MPC_Ctrllor(state,delta,path(idx:idxend,1:2),refHead(idx:idxend),Ts,Q,R,Np,Nc,rho);
    
    %更新车辆控制状态
    deltacmd=ddelta+delta;
    deltacmd=sign(deltacmd)*min([abs(deltacmd),30/180*pi]);
    U=deltacmd;
    LTR=observe(6);
    sys=[deltacmd,LTR,ctrlMode]; % steering
% End of mdlOutputs.
end
%% 子函数:MPC控制器
function [ddelta,observe]=MPC_Ctrllor(state,delta,refPos,refHead,dt,Q,R,Np,Nc,rho)
% 获得离散化线性模型和观测量
[A,B,C,observe]=TrailorTruck_5DOF_SP(state,dt);
%MPC控制器相关参数
x=state;
u=delta;
rate_delta=20/180*pi*dt;%限定方向盘转动速度
uconstrain=[-15*pi/180,15*pi/180, -rate_delta,rate_delta];
yconstrain=[-1e5,1e5;
            -1e5,1e5;
            -1e5,1e5;
            -1,1;
            -1,1;
            -0.7,0.7];
%参考轨迹
lenRef=size(refPos,1);
refPosx=interp1(1:lenRef,refPos(:,1),linspace(1,lenRef,Np));
refPosy=interp1(1:lenRef,refPos(:,2),linspace(1,lenRef,Np));
refHeads=interp1(1:lenRef,refHead(:,1),linspace(1,lenRef,Np));
Yr=reshape([refPosy;refPosx;refHeads;zeros(3,Np)],6*Np,1);
% Yr=kron(ones(Np,1),[refPos(1,1);refPos(1,2);refHead(1)]);
%获得前轮速度变化量、前轮转角变化量两个控制量
du=MPC_Controllor_qpOASES_Ycons(A,B,C,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho);

%获取相对参考量的控制变化量输出
ddelta=du;  

end
%% 子函数：输入单位转换*******************************
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18×1 Nx=18
function state=TruckSimInput(u)
vy1=u(1)/3.6;
df1=u(2)/180*pi;
F1=u(3)/180*pi;
dF1=u(4)/180*pi;
vy2=u(5)/3.6;
df2=u(6)/180*pi;
F2=u(7)/180*pi;
dF2=u(8)/180*pi;
th=u(9);
dth=u(10);
f1=u(11)/180*pi;
f2=u(12)/180*pi;
vx1=u(13)/3.6;
vx2=u(14)/3.6;
Y1=u(15);
X1=u(16);
Y2=u(17);
X2=u(18);

state=[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
      th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';%18×1 Nx=18
end
%% 子函数：getPath()*********************************
%获取参考路径
function [path,cur,len]=getPath(pathname)
    % pathname='S';%轨迹可选为单移线'SLC'或S型'S'
    %% 获得参考轨迹
    if strcmp(pathname,'CornerMain')
        load Corner_mainClass.mat len path cur
    elseif strcmp(pathname,'S')
        load S_path.mat path;
        cur=curvature(path(:,1),path(:,2));
        len=xy2distance(path(:,1),path(:,2));
    elseif strcmp(pathname,'DLC_trucksim')
        load DLCpathTruckSim.mat cv
        path=SplinePath(cv,1000);%点的间隔大概是350/1000=0.35m
        cur=curvature(path(:,1),path(:,2));
        len=xy2distance(path(:,1),path(:,2));
    else
        disp('请输入合法的换道轨迹名称呀');
    end
end
%% 子函数：获取参考轨迹最近的点
function [idx,idxend]=findTargetIdx(pos,path,len,spd,Ts,Np)
dist=zeros(size(path,1),1);
for i=1:size(dist,1)
   dist(i,1)=norm(path(i,1:2)-pos);
end
[~,idx]=min(dist); %找到距离当前位置最近的一个参考轨迹点的序号和距离
dist=abs(len-(len(idx)+spd*Ts*Np));
[~,idxend]=min(dist); %找到距离当前位置最近的一个参考轨迹点的序号和距离
end
%% 子函数：MPC控制器 使用qpOASES求解器
function du=MPC_Controllor_qpOASES(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,rho)
%% 模型处理 
%统计模型状态、控制量和观测量维度
Nx=size(a,1); %状态量个数
Nu=size(b,2); %控制量个数
Ny=size(c,1); %观测量个数
%构建控制矩阵
A=[a,b;zeros(Nu,Nx),eye(Nu)]; %(Nx+Nu) x (Nx+Nu)
B=[b;eye(Nu)];                %(Nx+Nu) x Nu
C=[c zeros(Ny,Nu)];           %   Ny   x (Nx+Nu)
%新的控制量为ksai(k)=[x(k),u(k-1)]'
ksai=[x;u];
%新的状态空间表达式为：ksai(k+1)=A*ksai(k)+B*du(k)  
%输出方程为： ita(k)=C*ksai(k)   %Ny x 1

%% 预测输出
% 获取相关预测矩阵
psai=zeros(Ny*Np,Nx+Nu); %矩阵psai
for i=1:Np
    psai(((i-1)*Ny+1):i*Ny,:)=C*A^i;
end
theta=zeros(Np*Ny,Nc*Nu); %矩阵theta
for i=1:Np
   for j=1:i
       if j<=Nc
       theta(((i-1)*Ny+1):i*Ny,((j-1)*Nu+1):j*Nu)=C*(A^(i-j))*B;
       else
       end
   end
end
%输出方程可以写为 Y=psai*ksai(k)+theta*dU  % Ny*Np x 1

%% 控制
% 变量设置
E=psai*ksai;
Qq=kron(eye(Np),Q);
Rr=kron(eye(Nc),R);
% 目标函数设计
% H=theta'*Qq*theta+Rr;
H=[theta'*Qq*theta+Rr,zeros(Nu*Nc,1);zeros(1,Nu*Nc),rho];
H=(H+H')/2;%保证矩阵对称
g=[(E'*Qq*theta - Yr'*Qq*theta)';0];
% 约束条件相关矩阵
At_tmp=zeros(Nc); %下三角方阵
for i=1:Nc
    At_tmp(i,1:i)=1;
end
At=[kron(At_tmp,eye(Nu)),zeros(Nu*Nc,1)];
%控制量及其变化量的限制
Umin=kron(ones(Nc,1),uconstrain(:,1));
Umax=kron(ones(Nc,1),uconstrain(:,2));
dUmin=[kron(ones(Nc,1),uconstrain(:,3));-1e10];
dUmax=[kron(ones(Nc,1),uconstrain(:,4));1e10];
%上一时刻的控制量
Ut=kron(ones(Nc,1),u);
%限制量矩阵：
% Acons=[At;-At];
% bcons=[Umax-Ut;-Umin+Ut];
%开始求解过程
% options=optimoptions('quadprog','MaxIterations',100,'TolFun',1e-16);
% dU=quadprog(H,g,Acons,bcons,[],[],dUmin,dUmax,[],options); %（Nu*Nc）x 1
options = qpOASES_options('default', 'printLevel', 0); 
% [dU, FVAL, EXITFLAG, iter, lambda] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options); %
[dU, ~, ~, ~, ~] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options);
du=dU(1:Nu);

end
%% 函数：MPC控制器：MPC_Controllor_qpOASES_Ycons*********************************
%****************************使用说明*****************************************************
% ●输入：
%   a,b,c:  为离散形式的模型A,B,C矩阵
%   Q,R:    最优调节的Q，R矩阵，Q为半正定，R为正定
%   x,u:    状态量x(k)和控制量u(k-1)
%   Np,Nc:  Np和Nc分别为预测时域和控制时域个数
%   uconstrain: 控制量及其变化量的限制，形式如下：
%               [u1min u1max du1min du1max;
%               u2min u2max du2min du2max];
%   yconstrain：为观测量，即系统输出的限制，可以设计为硬约束或者软约束，这里使用软约束。
%               假设观测量数量为3个，即Ny=3，则使用样例如下：
%               [y1min y1max;
%                y2min y2max;
%                y3min;y3max];
%   rho:    为松弛因子权重，大于0的数字，数值大表示限制松弛因子，即对输出量的约束更硬。
% ●输出:
%   du：    控制量的变化量，Nu x 1
%****************************************************************************************
% 调教MPC时可用以下参数初始化：
% a=rand(3);b=rand(3,2);c=eye(3);x=rand(3,1);u=[0;0];Q=eye(3);R=0.1*eye(2);rho=5;
% Np=50;Nc=3;Yr=zeros(Np*size(c,1),1);uconstrain=[-1 1 -0.1 0.1; -2 2 -0.2 0.2];
% yconstrain=[-0.1,0.1;-1,1;-1,1]*0.01;

function du=MPC_Controllor_qpOASES_Ycons(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho)
%% 模型处理 
%统计模型状态、控制量和观测量维度
Nx=size(a,1); %状态量个数
Nu=size(b,2); %控制量个数
Ny=size(c,1); %观测量个数
%构建控制矩阵
A=[a,b;zeros(Nu,Nx),eye(Nu)]; %(Nx+Nu) x (Nx+Nu)
B=[b;eye(Nu)];                %(Nx+Nu) x Nu
C=[c zeros(Ny,Nu)];           %   Ny   x (Nx+Nu)
%新的控制量为ksai(k)=[x(k),u(k-1)]'
ksai=[x;u];
%新的状态空间表达式为：ksai(k+1)=A*ksai(k)+B*du(k)  
%输出方程为： ita(k)=C*ksai(k)   %Ny x 1

%% 预测输出
% 获取相关预测矩阵
psai=zeros(Ny*Np,Nx+Nu); %矩阵psai
for i=1:Np
    psai(((i-1)*Ny+1):i*Ny,:)=C*A^i;
end
theta=zeros(Np*Ny,Nc*Nu); %矩阵theta
for i=1:Np
   for j=1:i
       if j<=Nc
       theta(((i-1)*Ny+1):i*Ny,((j-1)*Nu+1):j*Nu)=C*(A^(i-j))*B;
       else
       end
   end
end
%输出方程可以写为 Y=psai*ksai(k)+theta*dU  % Ny*Np x 1

%% 控制
% 变量设置
E=psai*ksai;
Qq=kron(eye(Np),Q);
Rr=kron(eye(Nc),R);
% 目标函数设计
% H=theta'*Qq*theta+Rr;
H=[theta'*Qq*theta+Rr,zeros(Nu*Nc,1);zeros(1,Nu*Nc),rho];
H=(H+H')/2;%保证矩阵对称
g=[(E'*Qq*theta - Yr'*Qq*theta)';0];
% 约束条件相关矩阵
At_tmp=zeros(Nc); %下三角方阵
for i=1:Nc
    At_tmp(i,1:i)=1;
end
At=[kron(At_tmp,eye(Nu)),zeros(Nu*Nc,1);
                   theta,-ones(Ny*Np,1);
                   theta, ones(Ny*Np,1)];
%控制量及其变化量的限制
Umin=[kron(ones(Nc,1),uconstrain(:,1))];
Umax=[kron(ones(Nc,1),uconstrain(:,2))];
dUmin=[kron(ones(Nc,1),uconstrain(:,3));0];
dUmax=[kron(ones(Nc,1),uconstrain(:,4));1e3];
%上一时刻的控制量
Ut=kron(ones(Nc,1),u);
%输出量约束
Ymin=kron(ones(Np,1),yconstrain(:,1));
Ymax=kron(ones(Np,1),yconstrain(:,2));
%限制量矩阵：
% Acons=[At;-At];
% bcons=[Umax-Ut;-Umin+Ut];
%开始求解过程
% options=optimoptions('quadprog','MaxIterations',100,'TolFun',1e-16);
% dU=quadprog(H,g,Acons,bcons,[],[],dUmin,dUmax,[],options); %（Nu*Nc）x 1
options = qpOASES_options('default', 'printLevel', 0); 
% [dU, FVAL, EXITFLAG, iter, lambda] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options); %
[dU, ~, ~, ~, ~] = qpOASES(H, g, At, dUmin, dUmax, [Umin-Ut;ones(Ny*Np,1)*-1e10;Ymin-E], [Umax-Ut;Ymax-E;ones(Ny*Np,1)*1e10], options);
du=dU(1:Nu);

end

%% 函数：SplinePath()****************************
% 通过输入的插值点cv和点数ptNum，根据CV点的距离生成
% 尽可能均匀的插值。
function path = SplinePath(cv,ptNum)
t0=xy2distance(cv(:,1),cv(:,2));
t=linspace(t0(1),t0(end),ptNum)';
path=[pchip(t0,cv(:,1),t),pchip(t0,cv(:,2),t)];
end

function [A,B,C,observe]=TrailorTruck_5DOF_SP(state0,Ts)
%% 初始状态定义
%% 定义动力学模型参数
persistent a b c d e                    %轴距纵向几何参数
persistent Tw1 Tw2                      %轮距横向几何参数
persistent h1 h2 hc1 hc2                %质心至侧倾轴线距离
                                        %铰接点至侧倾轴线距离
persistent m1 m2 m1s m2s g;g=9.806;
persistent I1zz I1xx I1xz I2zz I2xx I2xz %惯性参量
persistent k1 k2 k3                      %线性轮胎模型参数
persistent kr1 kr2 c1 c2 k12             %悬架模型参数
persistent m0 mp h0 hp lp co             %等效摆模型参数
if isempty(a)
%摆模型参数
x=[10890,1.5,0.13,0.97,0.13];
sim_param=getParam_SP(x);
% sim_param=[m0,m1,h0+0.77,hp+0.77,lp,g,coeff];
m0=sim_param(1);
mp=sim_param(2);
h0=sim_param(3);
hp=sim_param(4);
lp=sim_param(5);
co=sim_param(7);
%车辆模型参数
L1=(5+6.27)/2;L2=(10.15+11.5)/2;       %轴距平均
a=1.385;b=L1-a;c=5.635-a;e=5.5;d=L2-e; %纵向几何
Tw1=(2.03+1.863*2)/3;Tw2=1.863;        %轮距平均
hm1s=1.02; %牵引车质心高度
hm2s=1.58; %50%充液率时半挂车质心高度
hhitch=1.1;%交接点高度
hroll1=0.2314;hroll2=0.9630;           %侧倾中心高度
h1=hm1s-hroll1;h2=hm2s-hroll2;hc1=hhitch-hroll1;hc2=hhitch-hroll2;%高度几何
m1s=6310;m1=m1s+570+785*2;             %牵引车质量
% m2s=5925;m2=m2s+665*2;               %半挂车空载质量
m2s=20387;m2=m2s+665*2;
I1xx=6879;I1xz=130;I1zz=19665;         %牵引车惯量
% I2xx=9960;I2xz=0;I2zz=179992;        %半挂车惯量
I2xx=9960;I2xz=0;I2zz=331380;
%轮胎模型参数
% k1=-37e4;k2=-82e4;k3=-80e4;             %侧偏刚度
k1=-37e4;k2=-82e4;k3=-80e4;             %侧偏刚度
kr1=18.7e5;kr2=9.14e5;k12=68.3e5;       %侧倾刚度
c1=64.4e3;c2=197.7e3;                   %悬架等效阻尼
end
%% 定义线性动力学模型
dF2=state0(8);f1=state0(11);f2=state0(12);
vx1=state0(13);vx2=state0(14);
%线性车辆动力学模型
% M*dX = A0*X + B0*u
m14=-m1s*h1*c-I1xz;
m21=m1*vx1*hc1-m1s*h1*vx1;
m24=I1xx+2*m1s*h1^2-m1s*h1*hc1;
m55=m2*vx2*hc2-m2s*h2*vx2;
m58=I2xx+2*m2s*h2^2-m2s*h2*hc2;

M=[m1*vx1*c, I1zz, 0,   m14,     0,        0,   0,     0;
    m21,    -I1xz, 0,   m24,     0,        0,   0,     0;
   m1*vx1,      0, 0, -m1s*h1, m2*vx2,     0,   0, -m2s*h2;
     0,         0, 0,     0,  m2*vx2*e, -I2zz,  0, I2xz-m2s*h2*e;
     0,         0, 0,     0,    m55,    -I2xz,  0,    m58;
     1,    -c/vx1, 0, -hc1/vx1,  -1,   -e/vx2,  0, hc2/vx2;
     0,         0, 1,     0,     0,        0,   0,     0;
     0,         0, 0,     0,     0,        0,   1,     0   ];
 
 a11 = (c+a)*k1 + (c-b)*k2;
 a12 = a*(c+a)*k1/vx1 - b*(c-b)*k2/vx1 - m1*vx1*c;
 a22 = (a*k1-b*k2)*hc1/vx1 + (m1s*h1-m1*hc1)*vx1;
 a23 = m1s*g*h1 - kr1 -k12;
 a32 = (a*k1-b*k2)/vx1 - m1*vx1;
 a36 = -d*k3/vx2 - m2*vx2;
 a46 = -d*(e+d)*k3/vx2 - m2*vx2*e;
 a56 = (m2s*h2-m2*hc2)*vx2 - d*k3*hc2/vx2;
 a57 = m2s*g*h2 - kr2 -k12;
 %  X=   b1     df1    F1  dF1        b2  df2, F2   dF2 
 Acm0=[ a11,    a12,    0,   0,       0,   0,  0,    0;
   (k1+k2)*hc1, a22,  a23, -c1,       0,   0, k12,   0;
     k1+k2,     a32,    0,   0,      k3,  a36, 0,    0;
        0,       0,     0,   0, (e+d)*k3, a46, 0,    0;
        0,       0,    k12,  0,   k3*hc2, a56, a57, -c2;
        0,      -1,     0,   0,       0,   1,  0,    0;
        0,       0,     0,   1,       0,   0,  0,    0;
        0,       0,     0,   0,       0,   0,  0,    1   ];
    
 Bcm0=[-(c+a)*k1, -k1*hc1, -k1, 0, 0, 0, 0, 0]';
 

 %       1    2   3    4    5    6    7    8   9  10
%  X = vy1,  df1,F1, dF1, vy2, df2,  F2, dF2, th dth
% dX = dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;dth;ddth ←M_
M_=zeros(10);
M_(9,9)=1;
M_(10,:)=[0,0,0,0,vx2/lp,0,0,hp/lp,0,1];
M_(3,:)=[0,0,0,0,(mp+m0)*vx2,0,(m0*h0+mp*(hp+lp))*dF2^2,-(m0+h0+mp*hp),0,mp*lp];
M_(4,:)=M_(3,:)*e;
M_(5,:)=M_(3,:)*hc2+...
[0,0,0,0,-(m0*h0+mp*hp)*vx2,0,0,(m0+h0^2+mp*hp^2),0,-mp*lp*hp];

A_=zeros(10);
A_(9,10)=1;
A_(10,:)=[0,0,0,0,0,-vx2/lp,0,0,-(hp/lp+1)*dF2^2-g/lp,-co];
A_(3,:)=[0,0,0,0,0,-(mp+m0)*vx2,0,0,0,0];
A_(4,:)=A_(3,:)*e;
A_(5,:)=A_(3,:)*hc2+...
[0,0,0,0,0,(m0*h0+mp*hp)*vx2,0,0,-mp*lp*g,0];
%       1    2   3    4    5    6    7    8   9  10
%  X =  0,   0,  0,   0,   0, df2,   0,   0, th  0
% dX =  0;   0;  0;   0;dvy2;   0;dF2;ddF2;dth;  0
 Acm=([M,zeros(8,2);zeros(2,10)]+M_)\([Acm0,zeros(8,2);zeros(2,10)]+A_);
 Bcm=([M,zeros(8,2);zeros(2,10)]+M_)\[Bcm0;zeros(2,1)];
 
 %% 增广状态量 并转变bi为vyi
 
 Trans=diag([vx1,1,1,1,vx2,1,1,1,1,1]);
 Ac=[Trans*Acm,zeros(10,8);            %增广状态量后的矩阵
     0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0;
     zeros(2,18);
     cos(f1),0,0,0,0,0,0,0,0,0,0,0,sin(f1),0,0,0,0,0;
    -sin(f1),0,0,0,0,0,0,0,0,0,0,0,cos(f1),0,0,0,0,0;
     0,0,0,0,cos(f2),0,0,0,0,0,0,0,0,sin(f2),0,0,0,0;
    0,0,0,0,-sin(f2),0,0,0,0,0,0,0,0,cos(f2),0,0,0,0  ];

 Bc=[Trans*Bcm;zeros(8,1)] ;
 
 %% 离散化线性模型
 [A,B]=c2d_zoh(Ac,Bc,Ts);             %离散化线性模型
 
%       1   2    3    4   5    6    7    8    9  10   11  12  13  14 15 16
% dX=[dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;df1;df2;dvx1;dvx2;dY1;dX2;dY2;dX2];
%维度：14×1 

%% 观测量计算
%观测量为 Y1 X1 f1 th dth LTR
C=[zeros(2,14),eye(2),zeros(2,2); % Y1 X1
         zeros(1,10),1,zeros(1,7);      % f1
         zeros(2,8),eye(2),zeros(2,8);  % th dth
  2/(mean([Tw1,Tw2])*(m1+m2)*g)*...
             [0,0,-kr1,-c1,0,0,-kr2,-c2],zeros(1,10);%LTR
         ];
observe=C*state0;

end
%% 根据变量获取参数
function sim_param=getParam_SP(x)
%根据变量生成传输参数
TotMass=14462;
m1=x(1);%参数1
m0=TotMass-m1;
h0=x(2);%参数2
hp=x(3);%参数3
lp=x(4);%参数4
g=9.806;
coeff=x(5);%参数5
exh=0.77;% 额外的高度是从罐底到简化力的中心：车轴心 的距离
sim_param=[m0,m1,h0+exh,hp+exh,lp,g,coeff];
end