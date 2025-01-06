clc
clear
% close all
%% �ο�˫���߹켣����
%������ֵ
shape=1.5; %����ת����ҳ̶�
dx1=20;    %ʼ��ƽ���̶ȣ�Խ��Խƽ��
dx2=20;    %�س�ƽ���̶ȣ�Խ��Խƽ��
dy1=3.5;  %���ƻ�����ʼy��λ��
dy2=3.5;   %���ƻ�������y��λ��
Xs1=50.19; %���ƻ�����ʼ����
Xs2=130.46; %���ƻ�����������
X_phi=0:1:300; %����������������ٶ�x_dot����

%% ���ɲο��켣
z1=shape/dx1*(X_phi-Xs1)-shape/2;
z2=shape/dx2*(X_phi-Xs2)-shape/2;
Y_ref=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));
heading=atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-...
    dy2*(1./cosh(z2)).^2*(1.2/dx2));
path=[X_phi',Y_ref'];
len=xy2distance(X_phi,Y_ref);
%% ��ͼչʾ���
figure(1)
subplot(2,1,1);
plot(X_phi,Y_ref,LineWidth=2);
title('DLC�����ο�·��')
xlabel('��������/m');
ylabel('��������/m')
ylim([-5,9])
% axis equal
% grid on
subplot(2,1,2);
plot(len,heading.*180/pi);
title('DLC�����ο������')
xlabel('·�̳���/m');
ylabel('�Ƕ�/��');
% grid on

%% ����켣
save ConDLCpath.mat path len heading 




