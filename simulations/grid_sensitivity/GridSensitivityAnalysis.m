load("data_gridsensi.mat")
% 数据的顺序是Time,Fx,Fy,Fz,Mx,My,Mz
figure(1)
set(gcf,'color','white')
% subplot(3,1,1)
col = [2 3 4 6];
% col2 = 2:7;
% plot_one_case(col2,data_3_100_5_slow);
% hold on
% plot_one_case(col2,data_2_100_5_slow);
% plot_one_case(col2,data_2_150_5_slow);
% plot_one_case(col2,data_2_200_5_slow);
% plot_one_case(col2,data_1_200_5_slow);
% plot_one_case(col2,data_1_200_2_slow);
% plot_one_case(col2,data_1_200_2_fast);
% xlabel('time [s]',Interpreter='latex')
% ylabel('Force \& Moment [N]/[Nm]',Interpreter='latex')
% legend( '$F_x$', ...
%         '$F_y$', ...
%        '$F_z$', ...
%         '$M_x$', ...
%         '$M_y$', ...
%        '$M_z$', ...
%        'Box','on','EdgeColor','None',Interpreter='latex')
% hold off
% xlim([0,20])

Colors = {	[0 0 0],[0 0.4470 0.7410],[0.8500 0.3250 0.0980],[0.9290 0.6940 0.1250],...
    [0.4940 0.1840 0.5560],[0.4660 0.6740 0.1880],[0.3010 0.7450 0.9330]};
subplot(2,1,1)
time = 0.3:0.05:20;
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_3_100_5_slow,1)*100,'k',LineWidth=1.5);
hold on
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_2_100_5_slow,1)*100,'Color',"#0072BD",LineWidth=1.5);
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_2_150_5_slow,1)*100,'Color',"#D95319");
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_2_200_5_slow,1)*100,'Color',"#EDB120");
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_1_200_5_slow,1)*100,'Color',"#7E2F8E");
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_1_200_2_slow,1)*100,'Color',"#77AC30");
semilogy(time,calc_norm(time,col,data_3_100_5_slow,data_1_200_2_fast,1)*100,'Color',"#4DBEEE");
hold off
xlabel('time [s]',Interpreter='latex')
ylabel('error [\%]',Interpreter='latex')
% legend( 'case 1', ...
%         'case 2', ...
%        'case 3', ...
%        'case 4', ...
%        'case 5', ...
%        'case 6', ...
%        'case 7', ...
%        'Box','on','edgecolor','none','Orientation','vertical',Interpreter='latex')%,Location='bestoutside'

subplot(2,1,2)
error = [0,calc_norm(time,col,data_3_100_5_slow,data_2_100_5_slow,2),...
        calc_norm(time,col,data_3_100_5_slow,data_2_150_5_slow,2),...
        calc_norm(time,col,data_3_100_5_slow,data_2_200_5_slow,2),...
        calc_norm(time,col,data_3_100_5_slow,data_1_200_5_slow,2),...
        calc_norm(time,col,data_3_100_5_slow,data_1_200_2_slow,2),...
        calc_norm(time,col,data_3_100_5_slow,data_1_200_2_fast,2),...
        ]*100;
plot(error,'k','LineWidth',1)
hold on
for i = 1:length(error)
scatter(i,error(i),30,Colors{i},'filled')
end
hold off
x = 1:7; % 刻度位置
labels = {'rf:3 b:100\newline tw:5 vi:s','rf:2 b:100\newline tw:5 vi:s', 'rf:2 b:150\newline tw:5 vi:s',...
    'rf:2 b:200\newline tw:5 vi:s', 'rf:1 b:200\newline tw:5 vi:s', 'rf:1 b:200\newline tw:2 vi:s', 'rf:1 b:200\newline tw:2 vi:f'}; % 刻度标签
% 设置刻度位置和标签
xticks(x);
xticklabels(labels);
xlabel('case',Interpreter='latex')
ylabel('Average Error [\%]',Interpreter='latex')


print('GridSensitivity','-dpng','-r300')



function plot_one_case(col,data)
Colors = {[0 0.4470 0.7410],[0.8500 0.3250 0.0980],[0.9290 0.6940 0.1250],...
    [0.4940 0.1840 0.5560],[0.4660 0.6740 0.1880],[0.3010 0.7450 0.9330]};
time = data(:,1);
content = data(:,col);
for i = 1:length(col)
plot(time,content(:,i),'Color',Colors{i});
end
end

