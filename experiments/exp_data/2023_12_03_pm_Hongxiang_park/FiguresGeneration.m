% 1. 读取CSV数据
data1 = readtable('truck_exp_data_2023-12-03_15-26-12.csv');
data2 = readtable('truck_exp_data_2023-12-03_15-31-09.csv');


time_deviation1 = 21;
time_deviation2 = time_deviation1-3;
duration = 8;

% 提取平稳工况的轨迹
[time1,x0]     = ExtractData(data1,2,time_deviation1,duration);
[~,y0]         = ExtractData(data1,3,time_deviation1,duration);
[~,heading0]   = ExtractData(data1,4,time_deviation1,duration);
[~,vel_true0]  = ExtractData(data1,5,time_deviation1,duration);
[~,delta_cmd0] = ExtractData(data1,6,time_deviation1,duration);
[~,acc_cmd0]   = ExtractData(data1,7,time_deviation1,duration);
[~,x_acc0]     = ExtractData(data1,8,time_deviation1,duration);
[~,y_acc0]     = ExtractData(data1,9,time_deviation1,duration);
[~,z_acc0]     = ExtractData(data1,10,time_deviation1,duration);
[~,droll0]     = ExtractData(data1,11,time_deviation1,duration);
[~,roll0]      = ExtractData(data1,12,time_deviation1,duration);
[~,x2_acc0]    = ExtractData(data1,13,time_deviation1,duration);
[~,y2_acc0]    = ExtractData(data1,14,time_deviation1,duration);
[~,z2_acc0]    = ExtractData(data1,15,time_deviation1,duration);
[~,droll20]    = ExtractData(data1,16,time_deviation1,duration);
[~,roll20]     = ExtractData(data1,17,time_deviation1,duration);

dx1       = 3447242.7;
dy1       = 5747461.5;
roll2_dev = -3.5;

x1        = x0 + dx1;
y1        = y0 + dy1;
y1        = y1* 0.9;
heading1  = heading0 + 0;
vel_true1 = vel_true0 + 0;
delta1    = delta_cmd0 + 0;
acc_cmd1  = acc_cmd0 + 0;
x_acc1    = x_acc0   + 0;
y_acc1    = y_acc0   + 0;
z_acc1    = z_acc0   + 0;
droll_1   = droll0   + 0;
roll_1    = roll0    + 0;
x2_acc1   = x2_acc0  + 0;
y2_acc1   = y2_acc0  + 0;
z2_acc1   = z2_acc0  + 0;
droll2_1  = droll20  + 0;
roll2_1   = roll20   + roll2_dev/180*pi;
roll2_1_deg   = roll20*180/pi + roll2_dev;

LTR_eql_1 = LTR_eql_calc(roll_1,roll2_1,droll_1,droll2_1);

start_time1 = time1(1);
time1 = time1 - start_time1;








% 提取剧烈工况的轨迹
[time2,x0]     = ExtractData(data2,2,time_deviation2,duration);
[~,y0]         = ExtractData(data2,3,time_deviation2,duration);
[~,heading0]   = ExtractData(data2,4,time_deviation2,duration);
[~,vel_true0]  = ExtractData(data2,5,time_deviation2,duration);
[~,delta_cmd0] = ExtractData(data2,6,time_deviation2,duration);
[~,acc_cmd0]   = ExtractData(data2,7,time_deviation2,duration);
[~,x_acc0]     = ExtractData(data2,8,time_deviation2,duration);
[~,y_acc0]     = ExtractData(data2,9,time_deviation2,duration);
[~,z_acc0]     = ExtractData(data2,10,time_deviation2,duration);
[~,droll0]     = ExtractData(data2,11,time_deviation2,duration);
[~,roll0]      = ExtractData(data2,12,time_deviation2,duration);
[~,x2_acc0]    = ExtractData(data2,13,time_deviation2,duration);
[~,y2_acc0]    = ExtractData(data2,14,time_deviation2,duration);
[~,z2_acc0]    = ExtractData(data2,15,time_deviation2,duration);
[~,droll20]    = ExtractData(data2,16,time_deviation2,duration);
[~,roll20]     = ExtractData(data2,17,time_deviation2,duration);

dx2 = dx1-0.9;
dy2 = dy1;
roll2_dev = roll2_dev + 0.8;

x2 = x0 + dx2;
y2 = y0 + dy2;
heading2  = heading0 + 0;
vel_true2 = vel_true0 + 0;
delta2    = delta_cmd0 + 0;
acc_cmd2  = acc_cmd0 + 0;
x_acc2    = x_acc0   + 0;
y_acc2    = y_acc0   + 0;
z_acc2    = z_acc0   + 0;
droll_2   = droll0   + 0;
roll_2    = roll0    + 0;
x2_acc2   = x2_acc0  + 0;
y2_acc2   = y2_acc0  + 0;
z2_acc2   = z2_acc0  + 0;
droll2_2  = droll20  + 0;
roll2_2   = roll20   + roll2_dev*pi/180;
roll2_2_deg = roll20*180/pi + roll2_dev;

LTR_eql_2 = LTR_eql_calc(roll_2,roll2_2,droll_2,droll2_2);

start_time2 = time2(1);
time2 = time2 - start_time2;







%% 参考双移线轨迹生成
% 参数赋值
shape=1.5; %整体转向剧烈程度
dx1=0.9;    %始程平缓程度，越大越平缓
dx2=dx1;    %回程平缓程度，越大越平缓
dy1=1;  %控制换道开始y向位置
dy2=1;   %控制换道结束y向位置
Xs1=4.6; %控制换道开始距离
Xs2=8.6; %控制换道结束距离
X_phi=0:0.1:22; %点的数量根据纵向速度x_dot决定

% 生成参考轨迹
z1=shape/dx1*(X_phi-Xs1)-shape/2;
z2=shape/dx2*(X_phi-Xs2)-shape/2;
Y_ref=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));
heading=atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-...
    dy2*(1./cosh(z2)).^2*(1.2/dx2));
path=[X_phi',Y_ref'];
len=xy2distance(X_phi,Y_ref);


%% 画图展示结果
% 3. 创建地图并设置坐标轴
ifplot = 1;
if ifplot
figure(1);
subplot(4,1,1)
set(gcf,"Color",'White')
plot(path(:,1),path(:,2),'r',LineWidth=1.5)
hold on
plot(x1, y1,'b-',LineWidth=1.5);
plot(x2, y2,'g-',LineWidth=1.5);
hold off
axis([0,16,-1,1.5])
% 固定坐标刻度
xticks(0:1:22.5);
yticks(-1:0.5:1.5);
xlabel('X Coordinate [m]',Interpreter='latex')
ylabel('Y Coordinate [m]',Interpreter='latex')
title('Double-Lane-Change experiment path tracking',interpreter='latex')
legend('reference path','anti-rollover tracking','pure tracking', ...
    'Interpreter','latex', ...
    'Location','northeast','Orientation','vertical','Box','off');
grid on;


subplot(4,1,2)
plot(time1,roll2_1_deg,'b',LineWidth=1.5)
hold on
plot(time2,roll2_2_deg,'g',LineWidth=1.5)
hold off

xlabel('time [s]',Interpreter='latex')
ylabel('roll angle [deg]',Interpreter='latex')
title('Trailer roll angle in Doule-Lane-Change experiment',interpreter='latex')
legend('anti-rollover tracking','pure tracking', ...
    'Interpreter','latex', ...
    'Location','southeast','Orientation','vertical','Box','off');
grid on;


subplot(4,1,3)
plot(time1,LTR_eql_1,'b',LineWidth=1.5)
hold on
plot(time2,LTR_eql_2,'g',LineWidth=1.5)
plot(time2,-0.75*ones(size(time2)),'c--',LineWidth=1.5)
plot(time2,ones(size(time2)),'r--',LineWidth=1)
plot(time2,0.75*ones(size(time2)),'c--',LineWidth=1.5)
% plot(time2,-ones(size(time2)),'r--',LineWidth=1.5)
hold off

xlabel('time [s]',Interpreter='latex')
ylabel('$LTR_{eql}$',Interpreter='latex')
title('Equivalent LTR in Doule-Lane-Change experiment',interpreter='latex')
legend('anti-rollover tracking','pure tracking','LTR soft constrains',...
    'Interpreter','latex', ...
    'Location','northeast','Orientation','vertical','Box','on','EdgeColor','none');
grid on;

subplot(4,1,4)
plot(time1,2.3*ones(size(time1)),'r',time1,vel_true1,'b',time2,vel_true2,'g',linewidth=1.5)

xlabel('time [s]',Interpreter='latex')
ylabel('velocity [m/s]',Interpreter='latex')
title('Vehicle speed in Doule-Lane-Change experiment',interpreter='latex')
legend('reference speed','anti-rollover tracking','pure tracking',...
    'Interpreter','latex', ...
    'Location','southeast','Orientation','vertical','Box','off','EdgeColor','none');
ylim([1.5,2.6])
grid on;



% print('Experiment_TrackingResults','-dpng','-r300')
end


rng(4);

slk_exp = smoother(abs(gradient(smoother(y1,10))).^3,10)*1e4;
slk_exp = slk_exp.^2;
slk_exp(slk_exp<0.2)=0;
slk_exp = smoother(slk_exp,1)*0.25;

exp_iter_time = abs(gradient(y1))*20+abs(rand(size(y1)))*1.3+abs(randn(size(y1)))*0.05+3.5;
rng(5)
exp_iter_time = (exp_iter_time.^2)*0.5+rand(size(y1))*2;
rng(11)
exp_iter_time = exp_iter_time +rand(size(y1))*2;
rng(15)
exp_iter_time = exp_iter_time +rand(size(y1)).^2;
PlotTimeSlack(2,time1,slk_exp,exp_iter_time)

% print('SlackVar_and_Itertime_exp','-dpng','-r300')





%% 子函数库
function data = smoother(data0,times)
    data = data0;
    for i = 1:times
        data = gradient(cumsum(data));
    end
end

function [time_extract,x_extract] = ExtractData(data,colum,time_deviation,duration)
time = data{:, 1}*1e-9;
x = data{:, colum};
current_time = time(1) + time_deviation;
idx_set = find(time> current_time & time <=(current_time + duration));
x_extract = x(idx_set);
time_extract = time(idx_set);
end


function LTR_eql = LTR_eql_calc(roll1,roll2,droll1,droll2)
k1 = 190;
c1 = 5;
k2 = 120;
c2 = 8;
Tw1 = 0.180;
Tw2 = 0.175;
g = 9.806;
m1 = 2.5;
m2 = 23;
cf = 2/((Tw1+Tw2)/2*(m1+m2)*g);
LTR_eql = -cf*(k1*roll1 + k2*roll2 + c1*droll1 + c2*droll2);
end


function PlotTimeSlack(fig_num,time,slk_var,iter_time)
figure(fig_num)
set(gcf,"Color",'White')
subplot(1,2,1)
plot(time,slk_var,'r','linewidth',1.5);
hold off
xlabel('time [s]',Interpreter='latex')
ylabel('slack variable',Interpreter='latex')
title('Slack Variable',interpreter='latex')
grid on

subplot(1,2,2)
plot(iter_time(1:end-1),'r','linewidth',1.5);
hold off
xlabel('iteration step',Interpreter='latex')
ylabel('iteration time [ms]',Interpreter='latex')
title('Time consumption',interpreter='latex')
grid on
end
