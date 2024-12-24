%% ȫ�ֲ�������
fill=[25 40 50 60 75 90];
T0=[10 8 6 4 2 1000];
model={'SP','TP','TPSP','DMTP'};
% model={'SP'};
% model={'TP','TPSP','DMTP'};
%% �������ݿ�
load BestResult.mat BestResult
load Data_Gasoline_SinWave.mat DataSet
load Data_Gasoline.mat DataSet2
datatime=DataSet(:,1,1);
%% ��ʼ������洢�ռ�
%��Ӧ����Һ�ʣ�������ٶȣ����ڣ�ģ��
SineCompare=zeros(6,5,6,4);
%% ѭ���������й���
for modNum=1:length(model)
    for i=1:length(fill)
        for ay=1:5
            for j=1:length(T0)
               %������Ų�����չʾ
               num=((i-1)*25+(ay-1)*5+j);
               fprintf(['ģ�ͣ�' model{modNum} ' ��' num2str(num) '������,'])
               disp(['���ȣ�' num2str(round((150*(modNum-1)+num)/(150*4)*100,2)) '%'])
               counter=((i-1)*5+ay);
               % ���õ�ǰ�������
               T=T0(j);
               fillRate=fill(i);
               tao=0.05;
               % ���з���
               

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
               % ���汾�η�����
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
%% ������
save SineCompare2.mat SineCompare

%% �Ӻ����������˲�
function filteredData = DataFilter(data, time, tao)

% ���������
% data: һά��������
% time: ������������Ӧ��ʱ�����У���dataά����ͬ��
% tao: ���Ի��ڵ�ʱ�䳣��

% ���������
% filteredData: �������Ի��ں����������


% ������Ի��ڴ��ݺ���
num = 1;
den = [tao 1];
G = tf(num,den);

% ���������ݽ����˲�
filteredData = lsim(G,data,time);

end
%% �Ӻ�����plotCompare()
function plotCompare(datatime,simOut,Data,tao,modName,ay,T,fillRate)
figure(1)
subplot(3,1,1)
plot(simOut.time,simOut.Fy,datatime,DataFilter(Data(:,1),datatime,tao));
title([modName ' $F_y$ with $f=$' num2str(fillRate) '\% $a_y$ period = ' num2str(T) ' $s$, amplitude=' num2str(ay) ' $m/s^2$'],Interpreter="latex")
xlabel('time [s]',Interpreter="latex")
ylabel('$F_y$ / N',Interpreter="latex")
legend(modName,'CFD',Interpreter="latex")
subplot(3,1,2)
plot(simOut.time,simOut.Fz,datatime,DataFilter(Data(:,2),datatime,tao));
title([modName ' $F_z$ with $f=$' num2str(fillRate) '\% $a_y$ period = ' num2str(T) ' $s$, amplitude=' num2str(ay) ' $m/s^2$'],Interpreter="latex")
xlabel('time [s]',Interpreter="latex")
ylabel('$F_z$ / N',Interpreter="latex")
legend(modName,'CFD',Interpreter="latex")
subplot(3,1,3)
plot(simOut.time,simOut.Mx,datatime,DataFilter(Data(:,3),datatime,tao));
title([modName ' $M_x$ with $f=$' num2str(fillRate) '\% $a_y$ period = ' num2str(T) ' $s$, amplitude=' num2str(ay) ' $m/s^2$'],Interpreter="latex")
xlabel('time [s]',Interpreter="latex")
ylabel('$M_x$ / Nm',Interpreter="latex")
legend(modName,'CFD',Interpreter="latex")
name=[modName '_fillingRate_' num2str(fillRate) '%_ay_period' num2str(T) 's_amp_' num2str(ay) 'm_s2_' date '_' datestr(datetime, 'HH-MM-ss') '.jpg'];
saveas(gcf,['SimCompare_PendModels/' name])
end