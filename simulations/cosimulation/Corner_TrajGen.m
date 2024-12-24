%% 生成次主干路左转弯参考轨迹
% 道路级别：次主干道
% 道路宽度：3.75 m 
% 路口形式：双向六车道
% 测试要求：路面附着系数0.85，车速暂未确定
%% 生成换道轨迹
% 主干道
[path,len,cur]=CornerGen(15);
plotResult(path,len,cur)
save Corner_mainClass.mat path len cur
% 次主干道
[path,len,cur]=CornerGen(10);
plotResult(path,len,cur)
save Corner_midClass.mat path len cur
% 支路
[path,len,cur]=CornerGen(5);
plotResult(path,len,cur)
save Corner_subClass.mat path len cur

function  [path,len,cur]=CornerGen(R_road)
% 路缘半径：10m, 车道宽度：3.75m
% R_road=10;
d=3.75;
% 左转弯半径：R_road+d*2.5;(假设双向四车道)
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
path=SplinePath(cv0,450);%点的间隔大概是0.3m
cur=curvature(path(:,1),path(:,2));
len=xy2distance(path(:,1),path(:,2));

end


function plotResult(path,len,cur)
%% 画图展示结果
figure
subplot(2,1,1)
plot(path(:,1),path(:,2),'r','linewidth',2);
xlabel('横坐标x / m')
ylabel('纵坐标y / m')
title('左转弯参考轨迹')
axis equal
axis([0,80,0,80])
grid on
subplot(2,1,2)
plot(len,cur,'r','linewidth',2);
xlabel('路程s / m')
ylabel('参考路径曲率 / 1/m')
title('左转弯参考曲率')
grid on
end





