%% 全局参数设置
fill=[25 40 50 60 75 90];
T0=[10 8 6 4 2 1000];
model={'SP','TP','TPSP','DMTP'};
%% 加载数据库
load BestResult.mat BestResult
load Data_Gasoline_SinWave.mat DataSet
load Data_Gasoline.mat DataSet2
datatime=DataSet(:,1,1);
%% 初始化结果存储空间
%对应：充液率，侧向加速度，周期，模型
SineCompare=zeros(6,5,6,4);
%% 循环仿真所有工况
for modNum=1:4
    for i=1:length(fill)
        for ay=1:5
            for j=1:length(T0)
               %计算序号并进度展示
               num=((i-1)*25+(ay-1)*5+j);
               fprintf(['模型：' model{modNum} ' 第' num2str(num) '个工况,'])
               disp(['进度：' num2str(round((150*(modNum-1)+num)/(150*4)*100,2)) '%'])
               counter=((i-1)*5+ay);
               % 设置当前仿真参数
               T=T0(j);
               fillRate=fill(i);
               tao=0.05;
               % 进行仿真
               

               if T==1000
                   Data2=DataSet2(:,[2 3 4],counter)/0.75;
                   if modNum==4
                   [fitness,simOut]=myFitness2(BestResult{modNum}(i,:),datatime,Data2,kron([1 1 1],mean(Data2(:,2))),fillRate,ay,tao);
                   elseif modNum==3
                   [fitness,simOut]=myFitnessTPSP2(BestResult{modNum}(i,:),datatime,Data2,kron([1 1 1],mean(Data2(:,2))),fillRate,ay,tao);    
                   elseif modNum==2
                   [fitness,simOut]=myFitnessTP2(BestResult{modNum}(i,:),datatime,Data2,kron([1 1 1],mean(Data2(:,2))),fillRate,ay,tao);    
                   elseif modNum==1
                   [fitness,simOut]=myFitnessSP2(BestResult{modNum}(i,:),datatime,Data2,kron([1 1 1],mean(Data2(:,2))),fillRate,ay,tao);    
                   end
               else
                   Data=DataSet(:,[2 3 4],num)/0.75;
                   if modNum==4
                   [fitness,simOut]=myFitness(BestResult{modNum}(i,:),datatime,Data,kron([1 1 1],mean(Data(:,2))),fillRate,ay,T,tao);
                   elseif modNum==3
                   [fitness,simOut]=myFitnessTPSP(BestResult{modNum}(i,:),datatime,Data,kron([1 1 1],mean(Data(:,2))),fillRate,ay,T,tao);    
                   elseif modNum==2
                   [fitness,simOut]=myFitnessTP(BestResult{modNum}(i,:),datatime,Data,kron([1 1 1],mean(Data(:,2))),fillRate,ay,T,tao);    
                   elseif modNum==1
                   [fitness,simOut]=myFitnessSP(BestResult{modNum}(i,:),datatime,Data,kron([1 1 1],mean(Data(:,2))),fillRate,ay,T,tao);    
                   end
               end
               % 保存本次仿真结果
               disp(num2str(round(fitness)))
                close all
                if T==1000
                    T=inf;
                    Data=Data2;
                end
                
                plotCompare(datatime,simOut,Data,tao,model{modNum},ay,T,fillRate);
               SineCompare(i,ay,j,modNum)=fitness;
            end
        end
    end
end
%% 保存结果
save SineCompare2.mat SineCompare

%% 子函数：惯性滤波
function filteredData = DataFilter(data, time, tao)

% 输入参数：
% data: 一维数据序列
% time: 数据序列所对应的时间序列（与data维度相同）
% tao: 惯性环节的时间常数

% 输出参数：
% filteredData: 经过惯性环节后的数据序列


% 构造惯性环节传递函数
num = 1;
den = [tao 1];
G = tf(num,den);

% 对输入数据进行滤波
filteredData = lsim(G,data,time);

end
%% 子函数：plotCompare()
function plotCompare(datatime,simOut,Data,tao,modName,ay,T,fillRate)
figure(1)
subplot(3,1,1)
plot(simOut.time,simOut.Fy,datatime,DataFilter(Data(:,1),datatime,tao));
title([modName '充液' num2str(fillRate) '% 受迫晃动Fy—ay周期' num2str(T) 's，幅值：' num2str(ay) 'm/s2'])
xlabel('仿真时间 / s')
ylabel('侧向力Fy / N')
legend(modName,'CFD结果')
subplot(3,1,2)
plot(simOut.time,simOut.Fz,datatime,DataFilter(Data(:,2),datatime,tao));
title([modName '充液' num2str(fillRate) '% 受迫晃动Fz—ay周期' num2str(T) 's，幅值：' num2str(ay) 'm/s2'])
xlabel('仿真时间 / s')
ylabel('垂向力Fz / N')
legend(modName,'CFD结果')
subplot(3,1,3)
plot(simOut.time,simOut.Mx,datatime,DataFilter(Data(:,3),datatime,tao));
title([modName '充液' num2str(fillRate) '% 受迫晃动Mx—ay周期' num2str(T) 's，幅值：' num2str(ay) 'm/s2'])
xlabel('仿真时间 / s')
ylabel('侧倾力矩Mx / Nm')
legend(modName,'CFD结果')
name=[modName '充液' num2str(fillRate) '% 受迫晃动—ay周期' num2str(T) 's，幅值：' num2str(ay) 'ms2_' date '_' datestr(datetime, 'HH-MM-ss') '.jpg'];
saveas(gcf,['仿真对比图片/' name])
end