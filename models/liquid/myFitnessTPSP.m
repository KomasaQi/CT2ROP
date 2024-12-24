%TPSPģ�ͣ���8���������
% ���� m1    m2    h0  pendlen hp  lp   coeff1 coeff2
% lb = [100  100 -3.5  0.01  -0.5 0.01  0.05   0.05];%���ò�������
% ub = [800  800  3.5  1.5   1.5  1.5    0.5    0.5];%���ò�������
%      m1                  swlen               pendlen        coeff1
% [1023.97977147137,-0.124902746895330,0.644618286219594,0.0924378160372007]
%      m2                   h0                    hp           lp            coeff2
% [1320.92276070696,1.49999981814014,0.129420646171076,0.971091416581266,0.130086004325807]
% x=[1023.97977147137,0,1.5,0.644618286219594,0.129420646171076,0.971091416581266,0.0924378160372007,0.130086004325807];
% x=[0,1320.92276070696,1.5,0.644618286219594,0.129420646171076,0.971091416581266,0.0924378160372007,0.130086004325807];
% x=[512,660.5,0.763889]
function [fitness,simOut]=myFitnessTPSP(x,datatime,DataSet,MeanData,fillRate,ay,T,tao)
%% ���ݱ�����ȡ����
sim_param=getParam_TPSP(x,fillRate);
%% ѭ���������
% ���沢��ò���
x0=zeros(12,1);
Ts=0.0005;
tspan=0:Ts:10;
[t,y]=ode45(@(t,y) DMTPfcn4(t,y,sim_param,Ts,ay,T),tspan,x0);
% ����������
len=length(t);
Fy=zeros(len,1);
Fz=zeros(len,1);
Mx=zeros(len,1);
for i=1:length(t)
[Fy(i),Fz(i),Mx(i)]=calc_Fp_TPSP(y(i,:),sim_param);
end
% ��������ֵ
time=0:0.001:10;
% sideForce=interp1(t,Fy,time);
% verticalForce=interp1(t,Fz,time);
% xMoment=interp1(t,Mx,time);
% % ��ֵ��öԱ��õ�����
% sideForce0=interp1(datatime,DataSet(:,1),time);
% verticalForce0=interp1(datatime,DataSet(:,2),time);
% xMoment0=interp1(datatime,DataSet(:,3),time);
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
% ����������
err_sf=mean(((sideForce0-sideForce)/(MeanData(1)*1e-3)).^2);
err_vf=mean(((verticalForce0-verticalForce)/(MeanData(2)*1e-3)).^2);
err_mx=mean(((xMoment0-xMoment)/(MeanData(3)*1e-3)).^2);
err=mean([err_sf err_vf err_mx]);
%% ������Ӧ��
fitness=err;

end


function filteredData = DataFilter(data, time, tao)

% ���������
% data: һά��������
% time: ������������Ӧ��ʱ�����У���dataά����ͬ��
% tao: ���Ի��ڵ�ʱ�䳣��

% ���������
% filteredData: �������Ի��ں����������

% ������Ի��ڴ��ݺ���
num = 1;
den = [tao 1];
G = tf(num,den);

% ���������ݽ����˲�
filteredData = lsim(G,data,time);

end

%% ���ݱ�����ȡ����
function sim_param=getParam_TPSP(x,fillRate)
%���ݱ������ɴ������
% ���� m1    m2    h0  pendlen hp  lp   coeff1 coeff2
TotMass=1753.00870070310/50*fillRate;
m1=x(1);%����1
m2=x(2);%����2
m0=TotMass-m1-m2;
a=1.24;
b=0.9;
h0=x(3);%����3
pendlen=x(4);%����4
ap=pendlen*a;
bp=pendlen*b;
hp=x(5);%����5
lp=x(6);%����6
g=9.806;
coeff1=x(7);%����7
coeff2=x(8);%����8
sim_param=[m0,m1,m2,a,b,h0,hp,lp,ap,bp,g,coeff1,coeff2];
end
%% DMTP΢�ַ���
function dy=DMTPfcn4(t,y,sim_param,Ts,ay,T)
ddy=ay*sin(2*pi/T*t);
dy=zeros(12,1);
dy(1)=y(2);
dy(2)=ddy;
dy(3)=(dy(2)-y(3))/Ts;
dy(4)=y(5);
dy(5)=0;
dy(6)=(dy(5)-y(6))/Ts;
dy(7)=y(8);
dy(8)=calc_ddth1_TPSP(y,sim_param);
dy(9)=(dy(8)-y(9))/Ts;
dy(10)=y(11);
dy(11)=calc_ddth2_TPSP(y,sim_param);
dy(12)=(dy(11)-y(12))/Ts;
end
%% ������������
function [Fy,Fz,Mx] = calc_Fp_TPSP(state,param)
% sim_param=[m0,m1,m2,a,b,h0,hp,lp,ap,bp,g,coeff1,coeff2];
m0=param(1);
m1=param(2);
m2=param(3);
% a=param(4);
b=param(5);
h0=param(6);
hp=param(7);
lp=param(8);
ap=param(9);
bp=param(10);
g=param(11);
% co1=param(12);
% co2=param(13);

% y=state(1);
% dy=state(2);
ddy=state(3);
f=state(4);
df=state(5);
ddf=state(6);
th1=state(7);
dth1=state(8);
ddth1=state(9);
th2=state(10);
dth2=state(11);
ddth2=state(12);

%�����������д
s1=sin(th1);
c1=cos(th1);
sf=sin(f);
cf=cos(f);
s1sf=sin(th1)*sin(f);
s1cf=sin(th1)*cos(f);
c1sf=cos(th1)*sin(f);
c1cf=cos(th1)*cos(f);
s2=sin(th2+f);
c2=cos(th2+f);
L=lp+hp;


%����������� λ��ʸ��
R0=[-h0*sf,h0*cf];

R1=[- b*sf+ap*s1cf+bp*c1sf
    b*cf+ap*s1sf-bp*c1cf];

R2=[-L*sf+lp*s2
    L*cf-lp*c2];

ddRp0=[ddy-h0*cf*ddf+df^2*h0*sf
-h0*sf*ddf-h0*cf*df^2+g];

ddRp1=[b*sf*df^2+ddy-b*cf*ddf-ap*cf*s1*df^2-bp*c1*sf*df^2-ap*cf*s1*dth1^2-bp*c1*sf*dth1^2+bp*cf*c1*ddf+ap*cf*c1*ddth1-ap*sf*s1*ddf-bp*sf*s1*ddth1-2*ap*c1*sf*df*dth1-2*bp*cf*s1*df*dth1
-b*cf*df^2-b*sf*ddf+bp*cf*c1*df^2+bp*cf*c1*dth1^2-ap*sf*s1*df^2-ap*sf*s1*dth1^2+ap*cf*s1*ddf+bp*c1*sf*ddf+ap*c1*sf*ddth1+bp*cf*s1*ddth1+2*ap*cf*c1*df*dth1-2*bp*sf*s1*df*dth1+g];

ddRp2=[ddy+ddf*(-L*cf+lp*c2)+ddth2*(lp*c2)+df^2*(L*sf-lp*s2)+dth2^2*(-lp*s2)+df*dth2*(-2*lp*s2)
ddf*(-L*sf+lp*s2)+ddth2*(lp*s2)+df^2*(-L*cf+lp*c2)+dth2^2*(lp*c2)+df*dth2*(2*lp*c2)+g];

Fp=m0*ddRp0+m1*ddRp1+m2*ddRp2;
Fy=Fp(1)*cf-(-Fp(2))*sf;
Fz=(-Fp(2))*cf+Fp(1)*sf;
Mx0=(m0*(R0(1)*-ddRp0(2)+R0(2)*ddRp0(1)));
Mx1=(m1*(R1(1)*-ddRp1(2)+R1(2)*ddRp1(1)));
Mx2=(m2*(R2(1)*-ddRp2(2)+R2(2)*ddRp2(1)));

Mx=Mx0+Mx1+Mx2;

end
%% ���ddth1
function ddth1 = calc_ddth1_TPSP(state,param)
% sim_param=[m0,m1,m2,a,b,h0,hp,lp,ap,bp,g,coeff1,coeff2];
% m0=param(1);
% m1=param(2);
% m2=param(3);
% a=param(4);
b=param(5);
% h0=param(6);
% hp=param(7);
% lp=param(8);
ap=param(9);
bp=param(10);
g=param(11);
co1=param(12);
% co2=param(13);

% y=state(1);
% dy=state(2);
ddy=-state(3);
f=state(4);
df=state(5);
ddf=state(6);
th1=state(7);
dth1=state(8);
% ddth1=state(9);
% th2=state(10);
% dth2=state(11);
% ddth2=state(12);


%�����������д
s1=sin(th1);
c1=cos(th1);
sf=sin(f);
cf=cos(f);

ddth1=1/((ap*c1)^2+(bp*s1)^2)*(1/2*dth1^2*sin(2*th1)*(ap^2-bp^2)+g*(ap*c1*sf-bp*s1*cf)+ddf*(ap*bp-ap*b*c1)+df^2*(1/2*sin(2*th1)*(ap^2-bp^2)+bp*b*s1)+ddy*(ap*c1*cf+bp*s1*sf));
ddth1=ddth1-co1*dth1;

end
%% ���ddth2
function ddth2 = calc_ddth2_TPSP(state,param)
% sim_param=[m0,m1,m2,a,b,h0,hp,lp,ap,bp,g,coeff1,coeff2];
% m0=param(1);
% m1=param(2);
% m2=param(3);
% a=param(4);
% b=param(5);
% h0=param(6);
hp=param(7);
lp=param(8);
% ap=param(9);
% bp=param(10);
g=param(11);
% co1=param(12);
co2=param(13);

% y=state(1);
% dy=state(2);
ddy=state(3);
f=state(4);
df=state(5);
ddf=state(6);
% th1=state(7);
% dth1=state(8);
% ddth1=state(9);
th2=state(10);
dth2=state(11);
% ddth2=state(12);


%�����������д
s1=sin(th2);
c1=cos(th2);
s2=sin(th2+f);
c2=cos(th2+f);
L=lp+hp;

ddth2=-(ddy*lp*c2+ddf*(-lp*L*c1+lp^2)-df^2*lp*L*s1+g*lp*s2)/(lp^2);
ddth2=ddth2-co2*dth2;

end





