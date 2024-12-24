%% ���ɴ�����·��ת��ο��켣
% ��·���𣺴����ɵ�
% ��·��ȣ�3.75 m 
% ·����ʽ��˫��������
% ����Ҫ��·�渽��ϵ��0.85��������δȷ��
%% ���ɻ����켣
% ���ɵ�
[path,len,cur]=CornerGen(15);
plotResult(path,len,cur)
save Corner_mainClass.mat path len cur
% �����ɵ�
[path,len,cur]=CornerGen(10);
plotResult(path,len,cur)
save Corner_midClass.mat path len cur
% ֧·
[path,len,cur]=CornerGen(5);
plotResult(path,len,cur)
save Corner_subClass.mat path len cur

function  [path,len,cur]=CornerGen(R_road)
% ·Ե�뾶��10m, ������ȣ�3.75m
% R_road=10;
d=3.75;
% ��ת��뾶��R_road+d*2.5;(����˫���ĳ���)
R=R_road+d*3.5;

startpoint=[40,0];
endpoint=startpoint+R;
p=[0.5,0];
q=[0,0.5];
lambda=0.63;
cv=[startpoint;
    startpoint+p;
    startpoint+2*p;
    startpoint+[R,0]*lambda;
    endpoint-[0,R]*lambda;
    endpoint-2*q;
    endpoint-q;
    endpoint];
path0=Bspline(cv,[0,1],0.005,2,'nobase');
cv0=[0,0;
    30,0;
    startpoint-2*p;
    path0;
    endpoint+2*q;
    [endpoint(1),80]];
path=SplinePath(cv0,450);%��ļ�������0.3m
cur=curvature(path(:,1),path(:,2));
len=xy2distance(path(:,1),path(:,2));

end


function plotResult(path,len,cur)
%% ��ͼչʾ���
figure
subplot(2,1,1)
plot(path(:,1),path(:,2),'r','linewidth',2);
xlabel('������x / m')
ylabel('������y / m')
title('��ת��ο��켣')
axis equal
axis([0,80,0,80])
grid on
subplot(2,1,2)
plot(len,cur,'r','linewidth',2);
xlabel('·��s / m')
ylabel('�ο�·������ / 1/m')
title('��ת��ο�����')
grid on
end





