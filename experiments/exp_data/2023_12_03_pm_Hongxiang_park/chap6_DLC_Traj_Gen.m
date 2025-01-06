clc
clear
% close all
%% 参考双移线轨迹生成
%参数赋值
shape=1.5; %整体转向剧烈程度
dx1=20;    %始程平缓程度，越大越平缓
dx2=20;    %回程平缓程度，越大越平缓
dy1=3.5;  %控制换道开始y向位置
dy2=3.5;   %控制换道结束y向位置
Xs1=50.19; %控制换道开始距离
Xs2=130.46; %控制换道结束距离
X_phi=0:1:300; %点的数量根据纵向速度x_dot决定

%% 生成参考轨迹
z1=shape/dx1*(X_phi-Xs1)-shape/2;
z2=shape/dx2*(X_phi-Xs2)-shape/2;
Y_ref=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));
heading=atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-...
    dy2*(1./cosh(z2)).^2*(1.2/dx2));
path=[X_phi',Y_ref'];
len=xy2distance(X_phi,Y_ref);
%% 画图展示结果
figure(1)
subplot(2,1,1);
plot(X_phi,Y_ref,LineWidth=2);
title('DLC换道参考路径')
xlabel('横向坐标/m');
ylabel('纵向坐标/m')
ylim([-5,9])
% axis equal
% grid on
subplot(2,1,2);
plot(len,heading.*180/pi);
title('DLC换道参考航向角')
xlabel('路程长度/m');
ylabel('角度/°');
% grid on

%% 保存轨迹
save ConDLCpath.mat path len heading 




