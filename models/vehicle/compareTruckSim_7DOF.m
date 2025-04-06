function compareTruckSim_7DOF(time,states,refStates,figID)
if nargin < 3
    figID = 15;
end
% 绘图用变量序号定义
var.v1y = 1;
var.dyaw1 = 2;
var.roll1 = 3;
var.droll1 = 4;
var.v2y = 5;
var.dyaw2 = 6;
var.roll2 = 7;
var.droll2 = 8;
var.th1 = 9;
var.dth1 = 10;
var.th2 = 11;
var.dth2 = 12;
var.v1x = 13;
var.yaw1 = 14;
var.Y1 = 15;
var.X1 = 16;
var.LTR = 17;
% 绘图用常用白底颜色定义
mycolor.blau = "#0072BD";
mycolor.rot = "#D95319";
mycolor.gelb = "#EDB120";
mycolor.lila = "#7E2F8E";
mycolor.grun = 	"#77AC30";
mycolor.hellblau = "#4DBEEE";
mycolor.dunkelrot = "#A2142F";

figure(figID)
subplot(5,2,1)
hold on
theVar = var.v1y;
coeff = 3.6;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.blau,LineStyle="--",LineWidth=1.5);
theVar = var.v2y;
coeff = 3.6;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.rot,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.rot,LineStyle="--",LineWidth=1.5);
hold off
xlabel('time [s]')
ylabel('vy [km/h]')
legend('v1y','v1y_{7DOF}','v2y','v2y_{7DOF}');

subplot(5,2,2)
hold on
theVar = var.droll1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.blau,LineStyle="--",LineWidth=1.5);
theVar = var.droll2;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.rot,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.rot,LineStyle="--",LineWidth=1.5);
hold off
xlabel('time [s]')
ylabel('droll [deg/s]')
legend('droll1','droll1_{7DOF}','droll2','droll2_{7DOF}');


subplot(5,2,3)
hold on
theVar = var.v1x;
coeff = 3.6;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.blau,LineStyle="--",LineWidth=1.5);

hold off
xlabel('time [s]')
ylabel('vx [km/h]')
legend('v1x','v1x_{7DOF}');

subplot(5,2,4)
hold on
theVar = var.yaw1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.lila,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.lila,LineStyle="--",LineWidth=1.5);

hold off
xlabel('time [s]')
ylabel('yaw [deg]')
legend('yaw1','yaw1_{7DOF}');


subplot(5,2,5)
hold on
theVar = var.roll1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.blau,LineStyle="--",LineWidth=1.5);
theVar = var.roll2;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.gelb,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.gelb,LineStyle="--",LineWidth=1.5);
hold off
xlabel('time [s]')
ylabel('roll [deg]')
legend('roll1','roll1_{7DOF}','roll2','roll2_{7DOF}');


subplot(5,2,6)
hold on
theVar = var.LTR;
coeff = 1;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.dunkelrot,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.dunkelrot,LineStyle="--",LineWidth=1.5);

hold off
xlabel('time [s]')
ylabel('LTR [/]')
legend('LTR','LTR_{7DOF}');


subplot(5,2,7)
hold on
theVar = var.dyaw1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.grun,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.grun,LineStyle="--",LineWidth=1.5);
theVar = var.dyaw2;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.lila,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.lila,LineStyle="--",LineWidth=1.5);
hold off
xlabel('time [s]')
ylabel('dyaw [deg/s]')
legend('dyaw1','dyaw1_{7DOF}','dyaw2','dyaw2_{7DOF}');

subplot(5,2,8)
hold on
theVar = var.th1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.gelb,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.gelb,LineStyle="--",LineWidth=1.5);
theVar = var.dth1;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.blau,LineStyle="--",LineWidth=1.5);
theVar = var.th2;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.lila,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.lila,LineStyle="--",LineWidth=1.5);
theVar = var.dth2;
coeff = 180/pi;
plot(time,refStates(:,theVar)*coeff,Color=mycolor.rot,LineStyle="-",LineWidth=2);
plot(time,states(:,theVar)*coeff,Color=mycolor.rot,LineStyle="--",LineWidth=1.5);
hold off
xlabel('time [s]')
ylabel('th or dth [deg or deg/s]')
legend('th1','th1_{7DOF}','dth1','dth1_{7DOF}','th2','th2_{7DOF}','dth2','dth2_{7DOF}');


subplot(5,2,9)
hold on

plot(refStates(:,var.X1),refStates(:,var.Y1),Color=mycolor.blau,LineStyle="-",LineWidth=2);
plot(states(:,var.X1),states(:,var.Y1),Color=mycolor.blau,LineStyle="--",LineWidth=1.5);

hold off
xlabel('X [m]')
ylabel('Y [m]')
legend('path','path_{7DOF}');
