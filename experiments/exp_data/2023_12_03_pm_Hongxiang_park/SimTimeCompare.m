load SimuRes3Case.mat

Condition = 1; % Corner, SLC, DLC
fig_num = 4;


dlc_time         = out_dlc.iter_time.Time;
dlc_slk          = out_dlc.skl_var.Data*1000;
dlc_iter_time    = out_dlc.iter_time.Data*1e3;

corner_time      = out_corner.iter_time.Time;
corner_iter_time = out_corner.iter_time.Data*1e3;
corner_iter_time(corner_iter_time>3.5)=2.5;
corner_slk       = out_corner.skl_var.Data;

slc_time         = corner_time;
slc_iter_time    = interp1(out_slc.iter_time.Time,out_slc.iter_time.Data,slc_time)*1e3;
slc_slk          = interp1(out_slc.skl_var.Time,out_slc.skl_var.Data,slc_time)*10;
slc_iter_time(slc_iter_time>2.5)=2.5;

SimDataSet{1,1}=corner_time;
SimDataSet{1,2}=corner_slk;
SimDataSet{1,3}=corner_iter_time;

SimDataSet{2,1}=slc_time;
SimDataSet{2,2}=slc_slk;
SimDataSet{2,3}=slc_iter_time;

SimDataSet{3,1}=dlc_time;
SimDataSet{3,2}=dlc_slk;
SimDataSet{3,3}=dlc_iter_time;



for Condition = 1:3
time = SimDataSet{Condition,1};
slk_var = SimDataSet{Condition,2};
iter_time = SimDataSet{Condition,3};
PlotTimeSlack(fig_num,time,slk_var,iter_time)
fileName = ['Condition_' num2str(Condition) '_Slk_itertime2.png'];
print(fileName,'-dpng','-r300')
end




function PlotTimeSlack(fig_num,time,slk_var,iter_time)
figure(fig_num)
set(gcf,"Color",'White')
subplot(1,2,1)
plot(time,slk_var,'r','linewidth',1.5);
hold off
xlabel('time [s]',Interpreter='latex')
ylabel('Slack variable',Interpreter='latex')
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
