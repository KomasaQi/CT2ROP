% 1. 读取CSV数据
data = readtable('truck_exp_data_2023-12-03_15-26-12.csv');
time = data{:, 1}*1e-9;
x = data{:, 2};
y = data{:, 3};
x = x+3447246;
y = y+5747461.5;

pre_time = 10;% s 轨迹中当前时刻前的时间
pone_time = 10; % s 轨迹中当前时刻后的时间

% 2. 提取前后10秒的数据
start_time = time(1);
end_time = time(end);

% 提取前10秒的数据
start_idx = find(time >= start_time & time <= start_time + pre_time);
x_before = x(start_idx);
y_before = y(start_idx);

% 提取后10秒的数据
end_idx = find(time >= end_time - 10 & time <= end_time);
x_after = x(end_idx);
y_after = y(end_idx);

% 3. 创建地图并设置坐标轴
f=figure;
map_plot = plot(x_before, y_before, 'b-', x_after, y_after, 'g-',LineWidth=1);
hold on;

% 设置坐标轴范围并相等
axis equal;
% axis([min(x) - 5, max(x) + 5, min(y) - 5, max(y) + 5]);
axis([0,22.5,-3,3.5])
% 固定坐标刻度
xticks(-0:1:22.5);
yticks(-3:1:3.5);
grid on;

% 4. 初始化动画并设置坐标轴
animation_fig = f;
% axis tight manual;
filename = 'traj_anime_smooth_long.mp4';
video_writer = VideoWriter(filename, 'MPEG-4');
video_writer.FrameRate = 20;% 设置视频帧率为20fps
open(video_writer);

% 5. 绘制动画
for i = 1:length(time)
    current_time = time(i);
    current_idx = find(time >= current_time - pre_time & time <= current_time + pone_time);
    % 绘制前10秒和后10秒的轨迹
    set(map_plot(1), 'XData', x(current_idx(1):i), 'YData', y(current_idx(1):i));
    set(map_plot(2), 'XData', x(i:current_idx(end)), 'YData', y(i:current_idx(end)));

    % 绘制当前点
    current_point = plot(x(i), y(i), 'ro', 'MarkerSize', 10,LineWidth=1);

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
