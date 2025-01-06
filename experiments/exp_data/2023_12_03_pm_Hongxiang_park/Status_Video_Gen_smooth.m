% 1. 读取CSV数据
data = readtable('truck_exp_data_2023-12-03_15-26-12.csv');
time = data{:, 1}*1e-9;
% roll = data{:, 12};
roll2 = data{:, 17}*180/pi-5;


pre_time = 10;% s 轨迹中当前时刻前的时间
pone_time = 10; % s 轨迹中当前时刻后的时间

% 2. 提取前后10秒的数据
start_time = time(1);
end_time = time(end);

% 提取前10秒的数据
start_idx = find(time >= start_time & time <= start_time + pre_time);
time_before = time(start_idx);
roll2_before = roll2(start_idx);

% 提取后10秒的数据
end_idx = find(time >= end_time - 10 & time <= end_time);
time_after = time(end_idx);
roll2_after = roll2(end_idx);

% 3. 创建地图并设置坐标轴
f=figure;
roll_plot = plot(time_before, roll2_before, 'b-', time_after, roll2_after, 'g-',LineWidth=1);
hold on;

% 设置坐标轴范围并相等

% axis([min(x) - 5, max(x) + 5, min(y) - 5, max(y) + 5]);

% 固定坐标刻度
% xticks(-0:1:22.5);
% yticks(-3:1:3.5);
grid on;

% 4. 初始化动画并设置坐标轴
animation_fig = f;
% axis tight manual;
filename = 'roll_anime_smooth_long.mp4';
video_writer = VideoWriter(filename, 'MPEG-4');
video_writer.FrameRate = 20;% 设置视频帧率为20fps
open(video_writer);

% 5. 绘制动画
for i = 1:length(time)
    current_time = time(i);
    current_idx = find(time >= current_time - pre_time & time <= current_time + pone_time);
    % 绘制前10秒和后10秒的轨迹
    set(roll_plot(1), 'XData', time(current_idx(1):i), 'YData', roll2(current_idx(1):i));
    set(roll_plot(2), 'XData', time(i:current_idx(end)), 'YData', roll2(i:current_idx(end)));
    xlim([current_time-pre_time,current_time+pone_time])
    ylim([-18,18])
    % 绘制当前点
    current_point = plot(time(i), roll2(i), 'ro', 'MarkerSize', 10,LineWidth=1);

    % 设置图像属性
    title(['Current Time: ' num2str(current_time)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    legend('Before 10s', 'After 10s', 'Current Point');
    grid on;

    % 写入动画帧
    frame = getframe(animation_fig);
    writeVideo(video_writer, frame);

    % 删除当前点图标，以便下一帧更新
    delete(current_point);
end

% 6. 关闭动画
close(video_writer);
