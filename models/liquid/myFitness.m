function [fitness,simOut]=myFitness(x,datatime,DataSet,MeanData,fillRate,ay,T,tao)
%% 根据变量获取参数
sim_param=getParam(x,fillRate);
%% 循环计算误差
% 仿真并获得参数
x0=zeros(15,1);
Ts=0.0005;
tspan=0:Ts:10;
[t,y]=ode45(@(t,y) DMTPfcn1(t,y,sim_param,Ts,ay,T),tspan,x0);
% 计算三向力
len=length(t);
Fy=zeros(len,1);
Fz=zeros(len,1);
Mx=zeros(len,1);
for i=1:length(t)
[Fy(i),Fz(i),Mx(i)]=calc_Fp(y(i,:),sim_param);
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
function sim_param=getParam(x,fillRate)
%根据变量生成传输参数
TotMass=1753.00870070310/50*fillRate;
m1=x(1);%参数1
m2=x(2);%参数2
m0=TotMass-m1-m2;
a=1.24;
b=0.9;
swlen=x(3);%参数3
a0=swlen*a;
b0=swlen*b;
pendlen=x(4);%参数4
ap=pendlen*a;
bp=pendlen*b;
r=x(5);%参数5
g=9.806;
coeff1=x(6);%参数6
coeff2=x(7);%参数7
sim_param=[m0,m1,m2,a0,b0,a,b,ap,bp,r,g,coeff1,coeff2];
end
%% DMTP微分方程
function dy=DMTPfcn1(t,y,sim_param,Ts,ay,T)
ddy=ay*sin(2*pi/T*t);
maximum=50*ones(15,1);
minimum=-50*ones(15,1);
dy=zeros(15,1);
dy(1)=y(2);
dy(2)=ddy;
dy(3)=(dy(2)-y(3))/Ts;
dy(4)=y(5);
dy(5)=0;
dy(6)=(dy(5)-y(6))/Ts;
dy(7)=y(8);
dy(8)=0;
dy(9)=(dy(8)-y(9))/Ts;
dy(10)=y(11);
dy(11)=calc_ddth1(y,sim_param);
dy(12)=(dy(11)-y(12))/Ts;
dy(13)=y(14);
dy(14)=calc_ddth2(y,sim_param);
dy(15)=(dy(14)-y(15))/Ts;
dy=max([dy';minimum'])';
dy=min([dy';maximum'])';
end
%% 求解三向力输出
function [Fy,Fz,Mx] = calc_Fp(state,param)
m0=param(1);
m1=param(2);
m2=param(3);
% a0=param(4);
b0=param(5);
% a=param(6);
b=param(7);
ap=param(8);
bp=param(9);
r=param(10);
g=param(11);
% co1=param(12);
% co2=param(13);

% y=state(1);
% z=state(4);
f=state(7);
th1=state(10);
th2=state(13);

% dy=state(2);
% dz=state(5);
df=state(8);
dth1=state(11);
dth2=state(14);

ddy=state(3);
ddz=state(6);
ddf=state(9);
ddth1=state(12);
ddth2=state(15);

%定义运算符简写
s1=sin(th1);
c1=cos(th1);
sf=sin(f);
cf=cos(f);
s1sf=sin(th1)*sin(f);
s1cf=sin(th1)*cos(f);
c1sf=cos(th1)*sin(f);
c1cf=cos(th1)*cos(f);
s3=sin(f+th1+th2);
c3=cos(f+th1+th2);


%定义质量块的 位置矢量
R0=[-(b-b0)*sf,(b-b0)*cf];

R1=[- b*sf+ap*s1cf+bp*c1sf
    b*cf+ap*s1sf-bp*c1cf];

R2=[-b*sf+ap*s1cf+bp*c1sf+r*s3
     b*cf+ap*s1sf-bp*c1cf-r*c3];

ddRp0=[ddy+sf*(b-b0)*df^2-cf*(b-b0)*ddf
ddz-cf*(b-b0)*df^2-sf*(b-b0)*ddf+g];

ddRp1=[b*sf*df^2+ddy-b*cf*ddf-ap*cf*s1*df^2-bp*c1*sf*df^2-ap*cf*s1*dth1^2-bp*c1*sf*dth1^2+bp*cf*c1*ddf+ap*cf*c1*ddth1-ap*sf*s1*ddf-bp*sf*s1*ddth1-2*ap*c1*sf*df*dth1-2*bp*cf*s1*df*dth1
ddz-b*cf*df^2-b*sf*ddf+bp*cf*c1*df^2+bp*cf*c1*dth1^2-ap*sf*s1*df^2-ap*sf*s1*dth1^2+ap*cf*s1*ddf+bp*c1*sf*ddf+ap*c1*sf*ddth1+bp*cf*s1*ddth1+2*ap*cf*c1*df*dth1-2*bp*sf*s1*df*dth1+g];

ddRp2=[b*sf*df^2+ddy-b*cf*ddf+r*c3*(ddf+ddth1+ddth2)-r*s3*(df+dth1+dth2)^2-ap*cf*s1*df^2-bp*c1*sf*df^2-ap*cf*s1*dth1^2-bp*c1*sf*dth1^2+bp*cf*c1*ddf+ap*cf*c1*ddth1-ap*sf*s1*ddf-bp*sf*s1*ddth1-2*ap*c1*sf*df*dth1-2*bp*cf*s1*df*dth1
ddz-b*cf*df^2+r*c3*(df+dth1+dth2)^2-b*sf*ddf+r*s3*(ddf+ddth1+ddth2)+bp*cf*c1*df^2+bp*cf*c1*dth1^2-ap*sf*s1*df^2-ap*sf*s1*dth1^2+ap*cf*s1*ddf+bp*c1*sf*ddf+ap*c1*sf*ddth1+bp*cf*s1*ddth1+2*ap*cf*c1*df*dth1-2*bp*sf*s1*df*dth1+g];

Fp=m0*ddRp0+m1*ddRp1+m2*ddRp2;
Fy=Fp(1)*cf-(-Fp(2))*sf;
Fz=(-Fp(2))*cf+Fp(1)*sf;
Mx0=(m0*(R0(1)*-ddRp0(2)+R0(2)*ddRp0(1)));
Mx1=(m1*(R1(1)*-ddRp1(2)+R1(2)*ddRp1(1)));
Mx2=(m2*(R2(1)*-ddRp2(2)+R2(2)*ddRp2(1)));

Mx=Mx0+Mx1+Mx2;

end
%% 求解ddth1
function ddth1 = calc_ddth1(state,param)
% m0=param(1);
m1=param(2);
m2=param(3);
% a0=param(4);
% b0=param(5);
% a=param(6);
b=param(7);
ap=param(8);
bp=param(9);
r=param(10);
g=param(11);
co1=param(12);
% co2=param(13);

% y=state(1);
% z=state(4);
f=state(7);
th1=state(10);
th2=state(13);

% dy=state(2);
% dz=state(5);
df=state(8);
dth1=state(11);
dth2=state(14);

ddy=state(3);
ddz=state(6);
ddf=state(9);
% ddth1=state(12);
ddth2=state(15);

%定义运算符简写
s1=sin(th1);
c1=cos(th1);
s2=sin(th2);
c2=cos(th2);
s3=sin(f+th1+th2);
c3=cos(f+th1+th2);
s12=sin(th1+th2);
c12=cos(th1+th2);
sf1=sin(f+th1);
cf1=cos(f+th1);
sf_1=sin(f-th1);
cf_1=cos(f-th1);
s2t1=sin(2*th1);
c2t1=cos(2*th1);
s2t12=sin(2*th1+th2);
c2t12=cos(2*th1+th2);


k1a=((ap^2+bp^2)*(m1+m2) + 2*m2*r^2 + (ap^2- bp^2)*(m1+m2)*c2t1 + 2*(ap+bp)*m2*r*c2 + 2*(ap-bp)*m2*r*c2t12);
k1b=2*m2*r^2*ddf + 2*m2*r^2*ddth2 + 2*m2*r*s3*ddz + ap*m1*sf1*ddz + ap*m2*sf1*ddz + bp*m1*sf1*ddz + bp*m2*sf1*ddz + 2*g*m2*r*s3 + ap*g*m1*sf1 + ap*g*m2*sf1 + bp*g*m1*sf1 + bp*g*m2*sf1 + ap*m1*cf_1*ddy + ap*m2*cf_1*ddy + bp^2*m1*s2t1*df^2 + bp^2*m2*s2t1*df^2 + ap*m1*sf_1*ddz + ap*m2*sf_1*ddz + bp^2*m1*s2t1*dth1^2 + bp^2*m2*s2t1*dth1^2 + ap*g*m1*sf_1 + ap*g*m2*sf_1 + 2*ap*bp*m1*ddf + 2*ap*bp*m2*ddf + 2*m2*r*c3*ddy + ap*m1*cf1*ddy + ap*m2*cf1*ddy + bp*m1*cf1*ddy + bp*m2*cf1*ddy + 2*ap*m2*r*c2*ddf + 2*bp*m2*r*c2*ddf + ap*m2*r*c2*ddth2 + bp*m2*r*c2*ddth2 + 2*bp*m2*r*s2t12*df^2 + 2*bp*m2*r*s2t12*dth1^2 + bp*m2*r*s2t12*dth2^2 + ap*m2*r*c2t12*ddth2 + 2*bp*m2*r*s2t12*df*dth2 + 2*bp*m2*r*s2t12*dth1*dth2;
%下面的减去上面的除最上面的
k1c=bp*m1*cf_1*ddy + bp*m2*cf_1*ddy + ap^2*m1*s2t1*df^2 + ap^2*m2*s2t1*df^2 + bp*m1*sf_1*ddz + bp*m2*sf_1*ddz + ap^2*m1*s2t1*dth1^2 + ap^2*m2*s2t1*dth1^2 + bp*g*m1*sf_1 + bp*g*m2*sf_1 + 2*b*bp*m1*s1*df^2 + 2*b*bp*m2*s1*df^2 + 2*ap*b*m1*c1*ddf + 2*ap*b*m2*c1*ddf + ap*m2*r*s2*dth2^2 + bp*m2*r*s2*dth2^2 + 2*b*m2*r*s12*df^2 + 2*b*m2*r*c12*ddf + 2*ap*m2*r*s2t12*df^2 + 2*ap*m2*r*s2t12*dth1^2 + ap*m2*r*s2t12*dth2^2 + bp*m2*r*c2t12*ddth2 + 2*ap*m2*r*s2t12*df*dth2 + 2*ap*m2*r*s2t12*dth1*dth2 + 2*ap*m2*r*s2*df*dth2 + 2*bp*m2*r*s2*df*dth2 + 2*ap*m2*r*s2*dth1*dth2 + 2*bp*m2*r*s2*dth1*dth2;
ddth1=(k1c-k1b)/k1a-dth1*co1;
end
%% 求解ddth2
function ddth2 = calc_ddth2(state,param)
% m0=param(1);
% m1=param(2);
% m2=param(3);
% a0=param(4);
% b0=param(5);
% a=param(6);
b=param(7);
ap=param(8);
bp=param(9);
r=param(10);
g=param(11);
% co1=param(12);
co2=param(13);

% y=state(1);
% z=state(4);
f=state(7);
th1=state(10);
th2=state(13);

% dy=state(2);
% dz=state(5);
df=state(8);
dth1=state(11);
dth2=state(14);

ddy=state(3);
ddz=state(6);
ddf=state(9);
ddth1=state(12);
% ddth2=state(15);

%定义运算符简写
s1=sin(th1);
c1=cos(th1);
% s2=sin(th2);
% c2=cos(th2);
sf=sin(f);
cf=cos(f);
s3=sin(f+th1+th2);
c3=cos(f+th1+th2);

k2a=c3*ddy + s3*ddz + g*s3 + r*c3^2*ddf + r* ddth1 + r*s3^2*ddf + b*sf*c3*df^2 + ap*cf*c1*c3*ddth1 + ap*cf*s1*s3*ddf + bp*c1*sf*s3*ddf + ap*c1*sf*s3*ddth1 + bp*cf*s1*s3*ddth1 + bp*cf*c1*s3*df^2 + bp*cf*c1*s3*dth1^2 + bp*cf*c1*c3*ddf + 2*ap*cf*c1*s3*df*dth1;
k2b=b*cf*s3*df^2 + b*cf*c3*ddf + b*sf*s3*ddf + ap*sf*s1*s3*df^2 + ap*sf*s1*s3*dth1^2 + ap*sf*s1*c3*ddf + bp*sf*s1*c3*ddth1 + ap*cf*s1*c3*df^2 + bp*c1*sf*c3*df^2 + ap*cf*s1*c3*dth1^2 + bp*c1*sf*c3*dth1^2 + 2*ap*c1*sf*c3*df*dth1 + 2*bp*cf*s1*c3*df*dth1 + 2*bp*sf*s1*s3*df*dth1;
ddth2=(k2b-k2a)/r-dth2*co2;
end





