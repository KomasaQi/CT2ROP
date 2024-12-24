%% 加载数据集
load SineCompare2.mat SineCompare
%% 定义基本数据
T=[15 10 8 6 4 2];
ay=[1 2 3 4 5];
fill=[25 40 50 60 75 90];
model={'SP','TP','TPSP','DMTP'};
scfact=0.001;
SineCompare=SineCompare.^scfact;
%% 画图展示结果
%对应：充液率，侧向加速度，周期，模型
for i=1:6
    fillRate=fill(i);
    figure('Units','centimeter','Position',[15 10 25 15]);
    for modNum=1:4
        subplot(2,2,modNum);
        modName=model{modNum};
        [Ay,Tt]=meshgrid(ay,T);
        Err=reshape(SineCompare(i,:,:,modNum),5,6);
        Err=Err(:,[6 1:5]);
        if fillRate==75 && modNum==4
            Err(4,1)=1000.^scfact;
            Err(5,1)=1000.^scfact;
        end
        semilogx(0.1,1);hold on
        contourf(1./Tt,Ay,Err',[0,100,200,3000].^(scfact));
        plot([0.1 0.1],[1 5],'r--','linewidth',1.5)
        grid on
        title(['充液' num2str(fillRate) '%：' modName '模型在正弦激励下的相对误差map图'])
        xlabel('ay变化频率 / s-1')
        ylabel('ay幅值 / m/s2')
    end
    name=['充液' num2str(fillRate) '%：' '各模型在正弦激励下的相对误差map图.jpg'];
    saveas(gcf,name);
    close
end

% 
% %对应：充液率，侧向加速度，周期，模型
% for i=1:6
%     fillRate=fill(i);
%     figure
%     for modNum=1:4
%         modName=model{modNum};
%         [Ay,Tt]=meshgrid(ay,T);
%         Err=reshape(SineCompare(i,:,:,modNum),5,5);
%         surf(1./Tt,Ay,Err');
%         hold on
%         shading flat
%         title(['充液' num2str(fillRate) '%：' '各模型在正弦激励下的相对误差map图'])
%         xlabel('ay变化频率 / s-1')
%         ylabel('ay幅值 / m/s2')
%     end
%     hold off
%     name=['立体：充液' num2str(fillRate) '%：' '各模型在正弦激励下的相对误差map图.jpg'];
%     saveas(gcf,name);
%     close
% end
