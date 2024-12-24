
optimizer = TrajOptimizer_MinLTR('ltr_zeta',0.4,'ltr_omega_n',1.8);
startBdry = TrajBoundaryCondition('pos',[0,0],'heading',0,'spd',20,'acc',0);
finalBdry = TrajBoundaryCondition('pos',[80,3.5],'heading',0/180*pi,'spd',20,'acc',0);
timeDuration = 4;
% tic
% [traj,cost,LTR,optParams,traj_compare,cost_compare] = optimizer.lctrajopt(timeDuration,startBdry,finalBdry);
% toc
tic
testNum = 100;
for i = 1:testNum
    [traj,cost,LTR,optParams,traj_compare,cost_compare] = optimizer.lctrajopt_mex2(timeDuration,startBdry,finalBdry);
end
timePassed = toc/testNum;
disp(['最小LTR轨迹平均优化时间：' num2str(timePassed*1000) 'ms'])

figure(5)
% subplot(2,1,1)
% plot(traj_compare(:,2),traj_compare(:,3),'g','LineWidth',1)
% hold on
% plot(traj(:,2),traj(:,3),'r','LineWidth',1.5)
% hold off
% title('轨迹形状')
% xlabel('x坐标[m]')
% ylabel('y坐标[m]')
% legend('5阶多项式','防侧翻6阶多项式','Location','northwest')
% 
% subplot(2,1,2)
% plot(traj(:,1),LTR,'LineWidth',1.5)
% title('LTR变化')
% xlabel('时间[s]')
% ylabel('LTR')
subplot(2,1,1)
plot(traj_compare(:,2),traj_compare(:,3),'g','LineWidth',1)
hold on
plot(traj(:,2),traj(:,3),'r','LineWidth',1.5)
hold off
title('Trajectory')
xlabel('X coordinate [m]')
ylabel('Y coordinate [m]')
legend('5 order polyline','Anti-rollover 6 order polyline','Location','northwest')

subplot(2,1,2)
plot(traj(:,1),LTR,'LineWidth',1.5)
title('Simulated LTR Change')
xlabel('time [s]')
ylabel('LTR')
disp(['最优剩余自由度系数' num2str(optParams) ',最优||LTR||∞为' num2str(cost) ',比五次多项式降低' num2str(cost_compare-cost)])
