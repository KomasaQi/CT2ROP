slope_data = readtable('slopes.csv');
load('13-Jan-2024 22-34-25MPC_T1_SP_LTR仿真结果.mat');

cut_pre_time = 0;
cut_pone_time = 0;
total_time = 15;
scale_coeff1 = [0    4     8.5   9  10   12   15;
               0.5  1.2  1.2  0  1 0.2  0.1];
scale_coeff2 = [0    4   8.5  10   12   15;
               0.5  1.2  1.2  1.2  1.2 0.2];

rng(7)

simtime0 = SimOut.Theta.Time;
theta_sim0 = reshape(SimOut.Theta.Data(:,1,:),size(simtime0));

idx_set = (simtime0 >= cut_pre_time & simtime0 <= (total_time-cut_pone_time));

simtime = simtime0(idx_set);
theta_sim = theta_sim0(idx_set);

slope0 = slope_data{:,1};
time0 = (1:length(slope0))/25;

start_time = 497.53;
extension_rate = sqrt(2.2);
duration = 15/extension_rate;

idx_set = (time0 >= start_time & time0 <= (start_time+duration));
slope = slope0(idx_set);
time = (time0(idx_set)-start_time)*extension_rate;
state_coeff = interp1(scale_coeff1(1,:),scale_coeff1(2,:),time');

noise = 0.02*randn(size(slope));

start_time = 726.7;
extension_rate = sqrt(1.6);
duration = 15/extension_rate;
idx_set = (time0 >= start_time & time0 <= (start_time+duration));
slope2 = slope0(idx_set);
time2 = (time0(idx_set)-start_time)*extension_rate;

idx_set = (time2' >= 5 & time2' <= 7.5 & (abs(slope2)<(5/180*pi)));
slope2(idx_set)=22/180*pi;
idx_set = (time2' >= 8.5 & time2' <= 9.5 & (abs(slope2)<(8/180*pi)));
slope2(idx_set)=-22/180*pi;
state_coeff22 = interp1(scale_coeff2(1,:),scale_coeff2(2,:),time2');

figure(3)
set(gcf,"Color",'White')
plot(simtime,atan(theta_sim)*180/pi,'r','linewidth',1.5);
hold on
plot(time,atan(slope.*state_coeff+noise)*180/pi,'b','linewidth',1.5)
plot(time2,atan(slope2.*state_coeff22)*180/pi,'g','linewidth',1.5)
plot(time,25*ones(size(time)),'r--','linewidth',1)
plot(time,-25*ones(size(time)),'r--','linewidth',1)
hold off
xlabel('scaled time [s]',Interpreter='latex')
ylabel('lincline angle [deg]',Interpreter='latex')
xticks(0:1:15)
title('Liquid surface inclination comparisom',interpreter='latex')
legend('anti-rollover simulation','anti-rollover experiment','pure tracking experiment',...
    'NaN(no data)','Interpreter','latex', ...
    'Location','northeast','Orientation','vertical','Box','on','EdgeColor','none');

grid on;




