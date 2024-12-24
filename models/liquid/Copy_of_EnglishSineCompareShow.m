%% �������ݼ�
load SineCompare2.mat SineCompare
%% �����������
T=[15 10 8 6 4 2];
ay=[1 2 3 4 5];
fill=[25 40 50 60 75 90];
model={'SP','TP','TPSP','DMTP'};
scfact=0.001;
SineCompare=SineCompare.^scfact;
%% ��ͼչʾ���
%��Ӧ����Һ�ʣ�������ٶȣ����ڣ�ģ��
figure('Units','centimeter','Position',[15 10 25 15]);
for i=1:6
    fillRate=fill(7-i);
    
    for modNum=1:4
%         subplot(6,4,4*(i-1)+modNum);
        mgL=0.04;
        mgB=0.06;
        w=(1-5*mgL)/4;
        h=(1-7*mgB)/6;
        subplot('position', [mgL+(w+mgL)*(modNum-1) mgB+(h+mgB)*(i-1) w h]);
        modName=model{modNum};
        [Ay,Tt]=meshgrid(ay,T);
        Err=reshape(SineCompare(7-i,:,:,modNum),5,6);
        Err=Err(:,[6 1:5]);
        if fillRate==75 && modNum==4
            Err(4,1)=1000.^scfact;
            Err(5,1)=1000.^scfact;
        end
        semilogx(0.1,1);hold on
        contourf(1./Tt,Ay,Err',[0,100,200,3000].^(scfact));
        plot([0.1 0.1],[1 5],'r--','linewidth',1.5)
        grid on
         title(['Error Map of ' modName ': f=' num2str(fillRate) '%'])
        xlabel('a_y Frequency (Hz)')
        ylabel('a_y Amplitude (m/s2)')
        set(gca,'Fontsize',9)%���������СΪ14
    end
    name=['English��Һ' num2str(fillRate) '%��' '��ģ�������Ҽ����µ�������mapͼ.jpg'];
   
  

end
  saveas(gcf,'Ӣ�����ҶԱ�.jpg');
% 
% %��Ӧ����Һ�ʣ�������ٶȣ����ڣ�ģ��
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
%         title(['��Һ' num2str(fillRate) '%��' '��ģ�������Ҽ����µ�������mapͼ'])
%         xlabel('ay�仯Ƶ�� / s-1')
%         ylabel('ay��ֵ / m/s2')
%     end
%     hold off
%     name=['���壺��Һ' num2str(fillRate) '%��' '��ģ�������Ҽ����µ�������mapͼ.jpg'];
%     saveas(gcf,name);
%     close
% end
