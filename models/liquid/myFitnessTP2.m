%TP模型，共4个参数拟合
% 参数 m1 swlen pendlen coeff
% lb = [100  -0.5  0.01 0.05];%设置参数下限
% ub = [1753  1.5  1.5   1 ];%设置参数上限
function [fitness,simOut]=myFitnessTP(x,datatime,DataSet,MeanData,fillRate,ay,tao)
%% 根据变量获取参数
sim_param=getParam_TP(x,fillRate);

%% 循环计算误差

% 仿真并获得参数
x0=zeros(9,1);
Ts=0.0005;
tspan=0:Ts:10;
[t,y]=ode45(@(t,y) DMTPfcn2(t,y,sim_param,Ts,ay),tspan,x0);
% 计算三向力
len=length(t);
Fy=zeros(len,1);
Fz=zeros(len,1);
Mx=zeros(len,1);
for i=1:length(t)
[Fy(i),Fz(i),Mx(i)]=calc_Fp_TP(y(i,:),sim_param);
end
% 三向力插值
time=0:0.001:10;
sideForce=DataFilter(interp1(t,Fy,time),time,tao);
verticalForce=DataFilter(interp1(t,Fz,time),time,tao);
xMoment=DataFilter(interp1(t,Mx,time),time,tao);
sideForce0=DataFilter(interp1(datatime,DataSet(:,1),time),time,tao);
verticalForce0=DataFilter(interp1(datatime,DataSet(:,2),time),time,tao);
xMoment0=DataFilter(interp1(datatime,DataSet(:,3),time),time,tao);
simOut.time=time;
simOut.Fy=sideForce;
simOut.Fz=verticalForce;
simOut.Mx=xMoment;
% 计算均方误差
err_sf=mean(((sideForce0-sideForce)/(MeanData(1)*1e-3)).^2);
err_vf=mean(((verticalForce0-verticalForce)/(MeanData(2)*1e-3)).^2);
err_mx=mean(((xMoment0-xMoment)/(MeanData(3)*1e-3)).^2);
err=mean([err_sf err_vf err_mx]);


%% 计算适应度
fitness=err;

end


function filteredData = DataFilter(data, time, tao)

% 输入参数：
% data: 一维数据序列
% time: 数据序列所对应的时间序列（与data维度相同）
% tao: 惯性环节的时间常数

% 输出参数：
% filteredData: 经过惯性环节后的数据序列


% 构造惯性环节传递函数
num = 1;
den = [tao 1];
G = tf(num,den);

% 对输入数据进行滤波
filteredData = lsim(G,data,time);

end

%% 根据变量获取参数
function sim_param=getParam_TP(x,fillRate)
%根据变量生成传输参数
TotMass=1753.00870070310/50*fillRate;
m1=x(1);%参数1
m0=TotMass-m1;
a=1.24;
b=0.9;
swlen=x(2);%参数2
a0=swlen*a;
b0=swlen*b;
pendlen=x(3);%参数3
ap=pendlen*a;
bp=pendlen*b;
g=9.806;
coeff=x(4);%参数6
sim_param=[m0,m1,a0,b0,a,b,ap,bp,g,coeff];
end
%% DMTP微分方程
function dy=DMTPfcn2(t,y,sim_param,Ts,ay)
ddy=ay;
dy=zeros(9,1);
dy(1)=y(2);
dy(2)=ddy;
dy(3)=(dy(2)-y(3))/Ts;
dy(4)=y(5);
dy(5)=0;
dy(6)=(dy(5)-y(6))/Ts;
dy(7)=y(8);
dy(8)=calc_ddth_TP(y,sim_param);
dy(9)=(dy(8)-y(9))/Ts;
end
%% 求解三向力输出
function [Fy,Fz,Mx] = calc_Fp_TP(state,param)
% sim_param=[m0,m1,a0,b0,a,b,ap,bp,g,coeff];
m0=param(1);
m1=param(2);
% a0=param(3);
b0=param(4);
% a=param(5);
b=param(6);
ap=param(7);
bp=param(8);
g=param(9);
% co=param(10);

% y=state(1);
% dy=state(2);
ddy=state(3);
f=state(4);
df=state(5);
ddf=state(6);
th=state(7);
dth=state(8);
ddth=state(9);

%定义运算符简写
s1=sin(th);
c1=cos(th);
sf=sin(f);
cf=cos(f);
s1sf=sin(th)*sin(f);
s1cf=sin(th)*cos(f);
c1sf=cos(th)*sin(f);
c1cf=cos(th)*cos(f);



%定义质量块的 位置矢量
R0=[-(b-b0)*sf,(b-b0)*cf];

R1=[- b*sf+ap*s1cf+bp*c1sf
    b*cf+ap*s1sf-bp*c1cf];

ddRp0=[ddy+sf*(b-b0)*df^2-cf*(b-b0)*ddf
-cf*(b-b0)*df^2-sf*(b-b0)*ddf+g];

ddRp1=[b*sf*df^2+ddy-b*cf*ddf-ap*cf*s1*df^2-bp*c1*sf*df^2-ap*cf*s1*dth^2-bp*c1*sf*dth^2+bp*cf*c1*ddf+ap*cf*c1*ddth-ap*sf*s1*ddf-bp*sf*s1*ddth-2*ap*c1*sf*df*dth-2*bp*cf*s1*df*dth
-b*cf*df^2-b*sf*ddf+bp*cf*c1*df^2+bp*cf*c1*dth^2-ap*sf*s1*df^2-ap*sf*s1*dth^2+ap*cf*s1*ddf+bp*c1*sf*ddf+ap*c1*sf*ddth+bp*cf*s1*ddth+2*ap*cf*c1*df*dth-2*bp*sf*s1*df*dth+g];


Fp=m0*ddRp0+m1*ddRp1;
Fy=Fp(1)*cf-(-Fp(2))*sf;
Fz=(-Fp(2))*cf+Fp(1)*sf;
Mx0=(m0*(R0(1)*-ddRp0(2)+R0(2)*ddRp0(1)));
Mx1=(m1*(R1(1)*-ddRp1(2)+R1(2)*ddRp1(1)));

Mx=Mx0+Mx1;

end
%% 求解ddth1
function ddth = calc_ddth_TP(state,param)
% sim_param=[m0,m1,a0,b0,a,b,ap,bp,g,coeff];
% m0=param(1);
% m1=param(2);
% a0=param(3);
% b0=param(4);
% a=param(5);
b=param(6);
ap=param(7);
bp=param(8);
g=param(9);
co=param(10);

% y=state(1);
% dy=state(2);
ddy=-state(3);
f=state(4);
df=state(5);
ddf=state(6);
th=state(7);
dth=state(8);
% ddth=state(9);


%定义运算符简写
s1=sin(th);
c1=cos(th);
sf=sin(f);
cf=cos(f);

ddth=1/((ap*c1)^2+(bp*s1)^2)*(1/2*dth^2*sin(2*th)*(ap^2-bp^2)+g*(ap*c1*sf-bp*s1*cf)+ddf*(ap*bp-ap*b*c1)+df^2*(1/2*sin(2*th)*(ap^2-bp^2)+bp*b*s1)+ddy*(ap*c1*cf+bp*s1*sf));
ddth=ddth-co*dth;

end





