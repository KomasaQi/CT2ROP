%% *****************MPC����������*********************************
% ģ�ͣ� 5DOF��ҳ�+SP����=6DOFģ��
% ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18��1 Nx=18
% �۲�����observe=[Y1,X1,f1,th,dth,LTR]';6��1  Ny=6
% �����������������Լ��

%% ������ ���ݱ�־λflag���л��������巽��
% MPC_TrajTrackT1_TPSP_LTR_Ctrllor
function [sys,x0,str,ts] = MPC_Corner_TrajTrackT1_SP_LTR_Cons_Cosim_Ctrllor(t,x,u,flag)
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
sizes.NumDiscStates  = 6; % this parameter doesn't matter
sizes.NumOutputs     = 3; %[steering,LTR]
sizes.NumInputs      = 18;% ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                            th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18��1 Nx=18
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =1e-4*ones(6,1);   
global U; % store current ctrl vector: delta_m
U=0;
global path refHead len LTR ctrlMode modeCounter
modeCounter=100;
LTR=0;%��ʼLTR
ctrlMode=1;%����ģʽ��������1�����ֻ� �� 2���෭
[path,~,len]=getPath('CornerMain');
%��òο������
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
global path len refHead U LTR ctrlMode modeCounter
%% ����LTR��������ģʽ
if abs(LTR)<=0.5   %�����������LTR��ֵ
    modeCounter=modeCounter+1;
else 
    modeCounter=0;
end
if modeCounter>30 %����2s LTR��Ȩ��ֵС��0.5
    ctrlMode=1;  %�켣����+�ֻ�Ϊ��
    fprintf([num2str(t) 's�л�ģʽ���켣����\n'])
else
    ctrlMode=2;  %���෭Ϊ��
    fprintf([num2str(t) 's �л�ģʽ�����෭\n'])
end
%% ��ز�������
%MPC����������
% �۲�����observe=[Y1,X1,f1,th,dth,LTR]';6��1  Ny=6
if ctrlMode==1 %�켣����+�ֻ�
%     Q=diag([650,650,250,3,4,5]);
    Q=diag([4050,4050,4050,1,1,1]);
    R=200;  
else    % ���෭ģʽ
%     Q=diag([950,950,550,2,2,3]);
    Q=diag([4050,4050,4050,1,1,1]);
    R=200;  
end
Np=40;
Nc=3;
rho=180000;   %�ɳ�����ϵ��
Ts=0.05;   %���Ʋ�������λ��s
%% ������
% ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18��1 Nx=18
    %����µ�״̬����[ģ��״̬����������]����Nx+Nu��x 1
    state=TruckSimInput(u);
    delta=U;
    pos=[u(16),u(15)];
    % ��òο���
    [idx,idxend]=findTargetIdx(pos,path,len,state(13),Ts,Np);
    %MPC������
    [ddelta,observe]=MPC_Ctrllor(state,delta,path(idx:idxend,1:2),refHead(idx:idxend),Ts,Q,R,Np,Nc,rho);
    
    %���³�������״̬
    deltacmd=ddelta+delta;
    deltacmd=sign(deltacmd)*min([abs(deltacmd),30/180*pi]);
    U=deltacmd;
    LTR=observe(6);
    sys=[deltacmd,LTR,ctrlMode]; % steering
% End of mdlOutputs.
end
%% �Ӻ���:MPC������
function [ddelta,observe]=MPC_Ctrllor(state,delta,refPos,refHead,dt,Q,R,Np,Nc,rho)
% �����ɢ������ģ�ͺ͹۲���
[A,B,C,observe]=TrailorTruck_5DOF_SP(state,dt);
%MPC��������ز���
x=state;
u=delta;
rate_delta=20/180*pi*dt;%�޶�������ת���ٶ�
uconstrain=[-15*pi/180,15*pi/180, -rate_delta,rate_delta];
yconstrain=[-1e5,1e5;
            -1e5,1e5;
            -1e5,1e5;
            -1,1;
            -1,1;
            -0.7,0.7];
%�ο��켣
lenRef=size(refPos,1);
refPosx=interp1(1:lenRef,refPos(:,1),linspace(1,lenRef,Np));
refPosy=interp1(1:lenRef,refPos(:,2),linspace(1,lenRef,Np));
refHeads=interp1(1:lenRef,refHead(:,1),linspace(1,lenRef,Np));
Yr=reshape([refPosy;refPosx;refHeads;zeros(3,Np)],6*Np,1);
% Yr=kron(ones(Np,1),[refPos(1,1);refPos(1,2);refHead(1)]);
%���ǰ���ٶȱ仯����ǰ��ת�Ǳ仯������������
du=MPC_Controllor_qpOASES_Ycons(A,B,C,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho);

%��ȡ��Բο����Ŀ��Ʊ仯�����
ddelta=du;  

end
%% �Ӻ��������뵥λת��*******************************
% ״̬����state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';18��1 Nx=18
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
      th,dth,f1,f2,vx1,vx2,Y1,X1,Y2,X2]';%18��1 Nx=18
end
%% �Ӻ�����getPath()*********************************
%��ȡ�ο�·��
function [path,cur,len]=getPath(pathname)
    % pathname='S';%�켣��ѡΪ������'SLC'��S��'S'
    %% ��òο��켣
    if strcmp(pathname,'CornerMain')
        load Corner_mainClass.mat len path cur
    elseif strcmp(pathname,'S')
        load S_path.mat path;
        cur=curvature(path(:,1),path(:,2));
        len=xy2distance(path(:,1),path(:,2));
    elseif strcmp(pathname,'DLC_trucksim')
        load DLCpathTruckSim.mat cv
        path=SplinePath(cv,1000);%��ļ�������350/1000=0.35m
        cur=curvature(path(:,1),path(:,2));
        len=xy2distance(path(:,1),path(:,2));
    else
        disp('������Ϸ��Ļ����켣����ѽ');
    end
end
%% �Ӻ�������ȡ�ο��켣����ĵ�
function [idx,idxend]=findTargetIdx(pos,path,len,spd,Ts,Np)
dist=zeros(size(path,1),1);
for i=1:size(dist,1)
   dist(i,1)=norm(path(i,1:2)-pos);
end
[~,idx]=min(dist); %�ҵ����뵱ǰλ�������һ���ο��켣�����ź;���
dist=abs(len-(len(idx)+spd*Ts*Np));
[~,idxend]=min(dist); %�ҵ����뵱ǰλ�������һ���ο��켣�����ź;���
end
%% �Ӻ�����MPC������ ʹ��qpOASES�����
function du=MPC_Controllor_qpOASES(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,rho)
%% ģ�ʹ��� 
%ͳ��ģ��״̬���������͹۲���ά��
Nx=size(a,1); %״̬������
Nu=size(b,2); %����������
Ny=size(c,1); %�۲�������
%�������ƾ���
A=[a,b;zeros(Nu,Nx),eye(Nu)]; %(Nx+Nu) x (Nx+Nu)
B=[b;eye(Nu)];                %(Nx+Nu) x Nu
C=[c zeros(Ny,Nu)];           %   Ny   x (Nx+Nu)
%�µĿ�����Ϊksai(k)=[x(k),u(k-1)]'
ksai=[x;u];
%�µ�״̬�ռ���ʽΪ��ksai(k+1)=A*ksai(k)+B*du(k)  
%�������Ϊ�� ita(k)=C*ksai(k)   %Ny x 1

%% Ԥ�����
% ��ȡ���Ԥ�����
psai=zeros(Ny*Np,Nx+Nu); %����psai
for i=1:Np
    psai(((i-1)*Ny+1):i*Ny,:)=C*A^i;
end
theta=zeros(Np*Ny,Nc*Nu); %����theta
for i=1:Np
   for j=1:i
       if j<=Nc
       theta(((i-1)*Ny+1):i*Ny,((j-1)*Nu+1):j*Nu)=C*(A^(i-j))*B;
       else
       end
   end
end
%������̿���дΪ Y=psai*ksai(k)+theta*dU  % Ny*Np x 1

%% ����
% ��������
E=psai*ksai;
Qq=kron(eye(Np),Q);
Rr=kron(eye(Nc),R);
% Ŀ�꺯�����
% H=theta'*Qq*theta+Rr;
H=[theta'*Qq*theta+Rr,zeros(Nu*Nc,1);zeros(1,Nu*Nc),rho];
H=(H+H')/2;%��֤����Գ�
g=[(E'*Qq*theta - Yr'*Qq*theta)';0];
% Լ��������ؾ���
At_tmp=zeros(Nc); %�����Ƿ���
for i=1:Nc
    At_tmp(i,1:i)=1;
end
At=[kron(At_tmp,eye(Nu)),zeros(Nu*Nc,1)];
%����������仯��������
Umin=kron(ones(Nc,1),uconstrain(:,1));
Umax=kron(ones(Nc,1),uconstrain(:,2));
dUmin=[kron(ones(Nc,1),uconstrain(:,3));-1e10];
dUmax=[kron(ones(Nc,1),uconstrain(:,4));1e10];
%��һʱ�̵Ŀ�����
Ut=kron(ones(Nc,1),u);
%����������
% Acons=[At;-At];
% bcons=[Umax-Ut;-Umin+Ut];
%��ʼ������
% options=optimoptions('quadprog','MaxIterations',100,'TolFun',1e-16);
% dU=quadprog(H,g,Acons,bcons,[],[],dUmin,dUmax,[],options); %��Nu*Nc��x 1
options = qpOASES_options('default', 'printLevel', 0); 
% [dU, FVAL, EXITFLAG, iter, lambda] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options); %
[dU, ~, ~, ~, ~] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options);
du=dU(1:Nu);

end
%% ������MPC��������MPC_Controllor_qpOASES_Ycons*********************************
%****************************ʹ��˵��*****************************************************
% �����룺
%   a,b,c:  Ϊ��ɢ��ʽ��ģ��A,B,C����
%   Q,R:    ���ŵ��ڵ�Q��R����QΪ��������RΪ����
%   x,u:    ״̬��x(k)�Ϳ�����u(k-1)
%   Np,Nc:  Np��Nc�ֱ�ΪԤ��ʱ��Ϳ���ʱ�����
%   uconstrain: ����������仯�������ƣ���ʽ���£�
%               [u1min u1max du1min du1max;
%               u2min u2max du2min du2max];
%   yconstrain��Ϊ�۲�������ϵͳ��������ƣ��������ΪӲԼ��������Լ��������ʹ����Լ����
%               ����۲�������Ϊ3������Ny=3����ʹ���������£�
%               [y1min y1max;
%                y2min y2max;
%                y3min;y3max];
%   rho:    Ϊ�ɳ�����Ȩ�أ�����0�����֣���ֵ���ʾ�����ɳ����ӣ������������Լ����Ӳ��
% �����:
%   du��    �������ı仯����Nu x 1
%****************************************************************************************
% ����MPCʱ�������²�����ʼ����
% a=rand(3);b=rand(3,2);c=eye(3);x=rand(3,1);u=[0;0];Q=eye(3);R=0.1*eye(2);rho=5;
% Np=50;Nc=3;Yr=zeros(Np*size(c,1),1);uconstrain=[-1 1 -0.1 0.1; -2 2 -0.2 0.2];
% yconstrain=[-0.1,0.1;-1,1;-1,1]*0.01;

function du=MPC_Controllor_qpOASES_Ycons(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho)
%% ģ�ʹ��� 
%ͳ��ģ��״̬���������͹۲���ά��
Nx=size(a,1); %״̬������
Nu=size(b,2); %����������
Ny=size(c,1); %�۲�������
%�������ƾ���
A=[a,b;zeros(Nu,Nx),eye(Nu)]; %(Nx+Nu) x (Nx+Nu)
B=[b;eye(Nu)];                %(Nx+Nu) x Nu
C=[c zeros(Ny,Nu)];           %   Ny   x (Nx+Nu)
%�µĿ�����Ϊksai(k)=[x(k),u(k-1)]'
ksai=[x;u];
%�µ�״̬�ռ���ʽΪ��ksai(k+1)=A*ksai(k)+B*du(k)  
%�������Ϊ�� ita(k)=C*ksai(k)   %Ny x 1

%% Ԥ�����
% ��ȡ���Ԥ�����
psai=zeros(Ny*Np,Nx+Nu); %����psai
for i=1:Np
    psai(((i-1)*Ny+1):i*Ny,:)=C*A^i;
end
theta=zeros(Np*Ny,Nc*Nu); %����theta
for i=1:Np
   for j=1:i
       if j<=Nc
       theta(((i-1)*Ny+1):i*Ny,((j-1)*Nu+1):j*Nu)=C*(A^(i-j))*B;
       else
       end
   end
end
%������̿���дΪ Y=psai*ksai(k)+theta*dU  % Ny*Np x 1

%% ����
% ��������
E=psai*ksai;
Qq=kron(eye(Np),Q);
Rr=kron(eye(Nc),R);
% Ŀ�꺯�����
% H=theta'*Qq*theta+Rr;
H=[theta'*Qq*theta+Rr,zeros(Nu*Nc,1);zeros(1,Nu*Nc),rho];
H=(H+H')/2;%��֤����Գ�
g=[(E'*Qq*theta - Yr'*Qq*theta)';0];
% Լ��������ؾ���
At_tmp=zeros(Nc); %�����Ƿ���
for i=1:Nc
    At_tmp(i,1:i)=1;
end
At=[kron(At_tmp,eye(Nu)),zeros(Nu*Nc,1);
                   theta,-ones(Ny*Np,1);
                   theta, ones(Ny*Np,1)];
%����������仯��������
Umin=[kron(ones(Nc,1),uconstrain(:,1))];
Umax=[kron(ones(Nc,1),uconstrain(:,2))];
dUmin=[kron(ones(Nc,1),uconstrain(:,3));0];
dUmax=[kron(ones(Nc,1),uconstrain(:,4));1e3];
%��һʱ�̵Ŀ�����
Ut=kron(ones(Nc,1),u);
%�����Լ��
Ymin=kron(ones(Np,1),yconstrain(:,1));
Ymax=kron(ones(Np,1),yconstrain(:,2));
%����������
% Acons=[At;-At];
% bcons=[Umax-Ut;-Umin+Ut];
%��ʼ������
% options=optimoptions('quadprog','MaxIterations',100,'TolFun',1e-16);
% dU=quadprog(H,g,Acons,bcons,[],[],dUmin,dUmax,[],options); %��Nu*Nc��x 1
options = qpOASES_options('default', 'printLevel', 0); 
% [dU, FVAL, EXITFLAG, iter, lambda] = qpOASES(H, g, At, dUmin, dUmax, Umin-Ut, Umax-Ut, options); %
[dU, ~, ~, ~, ~] = qpOASES(H, g, At, dUmin, dUmax, [Umin-Ut;ones(Ny*Np,1)*-1e10;Ymin-E], [Umax-Ut;Ymax-E;ones(Ny*Np,1)*1e10], options);
du=dU(1:Nu);

end

%% ������SplinePath()****************************
% ͨ������Ĳ�ֵ��cv�͵���ptNum������CV��ľ�������
% �����ܾ��ȵĲ�ֵ��
function path = SplinePath(cv,ptNum)
t0=xy2distance(cv(:,1),cv(:,2));
t=linspace(t0(1),t0(end),ptNum)';
path=[pchip(t0,cv(:,1),t),pchip(t0,cv(:,2),t)];
end

function [A,B,C,observe]=TrailorTruck_5DOF_SP(state0,Ts)
%% ��ʼ״̬����
%% ���嶯��ѧģ�Ͳ���
persistent a b c d e                    %������򼸺β���
persistent Tw1 Tw2                      %�־���򼸺β���
persistent h1 h2 hc1 hc2                %�������������߾���
                                        %�½ӵ����������߾���
persistent m1 m2 m1s m2s g;g=9.806;
persistent I1zz I1xx I1xz I2zz I2xx I2xz %���Բ���
persistent k1 k2 k3                      %������̥ģ�Ͳ���
persistent kr1 kr2 c1 c2 k12             %����ģ�Ͳ���
persistent m0 mp h0 hp lp co             %��Ч��ģ�Ͳ���
if isempty(a)
%��ģ�Ͳ���
x=[10890,1.5,0.13,0.97,0.13];
sim_param=getParam_SP(x);
% sim_param=[m0,m1,h0+0.77,hp+0.77,lp,g,coeff];
m0=sim_param(1);
mp=sim_param(2);
h0=sim_param(3);
hp=sim_param(4);
lp=sim_param(5);
co=sim_param(7);
%����ģ�Ͳ���
L1=(5+6.27)/2;L2=(10.15+11.5)/2;       %���ƽ��
a=1.385;b=L1-a;c=5.635-a;e=5.5;d=L2-e; %���򼸺�
Tw1=(2.03+1.863*2)/3;Tw2=1.863;        %�־�ƽ��
hm1s=1.02; %ǣ�������ĸ߶�
hm2s=1.58; %50%��Һ��ʱ��ҳ����ĸ߶�
hhitch=1.1;%���ӵ�߶�
hroll1=0.2314;hroll2=0.9630;           %�������ĸ߶�
h1=hm1s-hroll1;h2=hm2s-hroll2;hc1=hhitch-hroll1;hc2=hhitch-hroll2;%�߶ȼ���
m1s=6310;m1=m1s+570+785*2;             %ǣ��������
% m2s=5925;m2=m2s+665*2;               %��ҳ���������
m2s=20387;m2=m2s+665*2;
I1xx=6879;I1xz=130;I1zz=19665;         %ǣ��������
% I2xx=9960;I2xz=0;I2zz=179992;        %��ҳ�����
I2xx=9960;I2xz=0;I2zz=331380;
%��̥ģ�Ͳ���
% k1=-37e4;k2=-82e4;k3=-80e4;             %��ƫ�ն�
k1=-37e4;k2=-82e4;k3=-80e4;             %��ƫ�ն�
kr1=18.7e5;kr2=9.14e5;k12=68.3e5;       %����ն�
c1=64.4e3;c2=197.7e3;                   %���ܵ�Ч����
end
%% �������Զ���ѧģ��
dF2=state0(8);f1=state0(11);f2=state0(12);
vx1=state0(13);vx2=state0(14);
%���Գ�������ѧģ��
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
% dX = dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;dth;ddth ��M_
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
 
 %% ����״̬�� ��ת��biΪvyi
 
 Trans=diag([vx1,1,1,1,vx2,1,1,1,1,1]);
 Ac=[Trans*Acm,zeros(10,8);            %����״̬����ľ���
     0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0;
     zeros(2,18);
     cos(f1),0,0,0,0,0,0,0,0,0,0,0,sin(f1),0,0,0,0,0;
    -sin(f1),0,0,0,0,0,0,0,0,0,0,0,cos(f1),0,0,0,0,0;
     0,0,0,0,cos(f2),0,0,0,0,0,0,0,0,sin(f2),0,0,0,0;
    0,0,0,0,-sin(f2),0,0,0,0,0,0,0,0,cos(f2),0,0,0,0  ];

 Bc=[Trans*Bcm;zeros(8,1)] ;
 
 %% ��ɢ������ģ��
 [A,B]=c2d_zoh(Ac,Bc,Ts);             %��ɢ������ģ��
 
%       1   2    3    4   5    6    7    8    9  10   11  12  13  14 15 16
% dX=[dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;df1;df2;dvx1;dvx2;dY1;dX2;dY2;dX2];
%ά�ȣ�14��1 

%% �۲�������
%�۲���Ϊ Y1 X1 f1 th dth LTR
C=[zeros(2,14),eye(2),zeros(2,2); % Y1 X1
         zeros(1,10),1,zeros(1,7);      % f1
         zeros(2,8),eye(2),zeros(2,8);  % th dth
  2/(mean([Tw1,Tw2])*(m1+m2)*g)*...
             [0,0,-kr1,-c1,0,0,-kr2,-c2],zeros(1,10);%LTR
         ];
observe=C*state0;

end
%% ���ݱ�����ȡ����
function sim_param=getParam_SP(x)
%���ݱ������ɴ������
TotMass=14462;
m1=x(1);%����1
m0=TotMass-m1;
h0=x(2);%����2
hp=x(3);%����3
lp=x(4);%����4
g=9.806;
coeff=x(5);%����5
exh=0.77;% ����ĸ߶��Ǵӹ޵׵����������ģ������� �ľ���
sim_param=[m0,m1,h0+exh,hp+exh,lp,g,coeff];
end