% 生成测试信号
t = linspace(0, 2*pi, 50); % 时间范围为[0, 2π]，共计算50个点
x = sin(3*t) + 0.1*randn(size(t)); % 添加高斯白噪声
 
% 设置滤波参数
fc = 4; % 截止频率为4Hz
fs = 1/min(diff(t)); % 取样频率
order = 6; % 滤波器阶数
 
% 创建低通滤波器
b = fir1(order, 0.5, 'low');
a = [1];
filtered_signal = filter(b, a, x);
 
% 显示原始信号与滤波后的结果
figure;
subplot(2,1,1);
plot(t, x);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
 
subplot(2,1,2);
plot(t, filtered_signal);
title('Filtered Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;