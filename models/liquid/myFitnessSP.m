%SPģ�ͣ���5���������
% ���� m1 h0 hp lp coeff
% lb = [100  -0.5 -0.5 0.01 0.05];%���ò�������
% ub = [1753 1.5  1.5  1.5   1 ];%���ò�������
function [fitness,simOut]=myFitnessSP(x,datatime,DataSet,MeanData,fillRate,ay,T,tao)
%% ���ݱ�����ȡ����
sim_param=getParam_SP(x,fillRate);

%% ѭ���������

% ���沢��ò���
x0=zeros(9,1);
Ts=0.0005;
tspan=0:Ts:10;
[t,y]=ode45(@(t,y) DMTPfcn3(t,y,sim_param,Ts,ay,T),tspan,x0);
% ����������
len=length(t);
Fy=zeros(len,1);
Fz=zeros(len,1);
Mx=zeros(len,1);
for i=1:length(t)
[Fy(i),Fz(i),Mx(i)]=calc_Fp_SP(y(i,:),sim_param);
end
% ��������ֵ
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
function sim_param=getParam_SP(x,fillRate)
%���ݱ������ɴ������
TotMass=1753.00870070310/50*fillRate;
m1=x(1);%����1
m0=TotMass-m1;
h0=x(2);%����2
hp=x(3);%����3
lp=x(4);%����4
g=9.806;
coeff=x(5);%����5
sim_param=[m0,m1,h0,hp,lp,g,coeff];
end
%% DMTP΢�ַ���
function dy=DMTPfcn3(t,y,sim_param,Ts,ay,T)
ddy=ay*sin(2*pi/T*t);
dy=zeros(9,1);
dy(1)=y(2);
dy(2)=ddy;
dy(3)=(dy(2)-y(3))/Ts;
dy(4)=y(5);
dy(5)=0;
dy(6)=(dy(5)-y(6))/Ts;
dy(7)=y(8);
dy(8)=calc_ddth_SP(y,sim_param);
dy(9)=(dy(8)-y(9))/Ts;
end
%% ������������
function [Fy,Fz,Mx] = calc_Fp_SP(state,param)
% sim_param=[m0,m1,h0,hp,lp,g,coeff];
m0=param(1);
m1=param(2);
h0=param(3);
hp=param(4);
lp=param(5);
g=param(6);
% co=param(7);

% y=state(1);
% dy=state(2);
ddy=state(3);
f=state(4);
df=state(5);
ddf=state(6);
th=state(7);
dth=state(8);
ddth=state(9);

%�����������д
sf=sin(f);
cf=cos(f);
s2=sin(th+f);
c2=cos(th+f);
L=lp+hp;
%����������� λ��ʸ��
R0=[-h0*sf,h0*cf];

R1=[-L*sf+lp*s2
    L*cf-lp*c2];

ddRp0=[ddy-h0*cf*ddf+df^2*h0*sf
-h0*sf*ddf-h0*cf*df^2+g];

ddRp1=[ddy+ddf*(-L*cf+lp*c2)+ddth*(lp*c2)+df^2*(L*sf-lp*s2)+dth^2*(-lp*s2)+df*dth*(-2*lp*s2)
ddf*(-L*sf+lp*s2)+ddth*(lp*s2)+df^2*(-L*cf+lp*c2)+dth^2*(lp*c2)+df*dth*(2*lp*c2)+g];

Fp=m0*ddRp0+m1*ddRp1;
Fy=Fp(1)*cf-(-Fp(2))*sf;
Fz=(-Fp(2))*cf+Fp(1)*sf;
Mx0=(m0*(R0(1)*-ddRp0(2)+R0(2)*ddRp0(1)));
Mx1=(m1*(R1(1)*-ddRp1(2)+R1(2)*ddRp1(1)));

Mx=Mx0+Mx1;

end
%% ���ddth
function ddth = calc_ddth_SP(state,param)
% sim_param=[m0,m1,h0,hp,lp,g,coeff];
% m0=param(1);
% m1=param(2);
% h0=param(3);
hp=param(4);
lp=param(5);
g=param(6);
co=param(7);

% y=state(1);
% dy=state(2);
ddy=state(3);
f=state(4);
df=state(5);
ddf=state(6);
th=state(7);
dth=state(8);
% ddth=state(9);


%�����������д
s1=sin(th);
c1=cos(th);
s2=sin(th+f);
c2=cos(th+f);
L=lp+hp;

ddth=-(ddy*lp*c2+ddf*(-lp*L*c1+lp^2)-df^2*lp*L*s1+g*lp*s2)/(lp^2);
ddth=ddth-co*dth;

end





