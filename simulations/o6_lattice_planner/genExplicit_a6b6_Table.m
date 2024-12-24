%% 并行离线计算生成用于查表的最小LTR轨迹参数a6 b6离线结果

% 假设5个状态量的取值范围和离散化的数量
% 状态量包括 [v0 x y Dth]
range = [10/3.6 120/3.6;    % 状态量v0的范围,小于10km/h的都不需要防侧翻功能
         20 110;    % 状态量x的范围
        -60 60;   % 状态量y的范围
         -90/180*pi 90/180*pi];  % 状态量Dth的范围

n_discrete = [40 80 161 31];% 每个状态量的离散化数量
% n_discrete = [4 8 51 5];% 每个状态量的离散化数量
tf = 4; % 默认规划4s的未来状态

% 离散化状态量
v0_seq = linspace(range(1,1), range(1,2), n_discrete(1));
x_seq = linspace(range(2,1), range(2,2), n_discrete(2));
y_seq0 = 1:((n_discrete(3)-1)/2);
y_seq1 = real(y_seq0.^1.2);
y_seq = [-y_seq1(end:-1:1),0,y_seq1]/max(y_seq1)*range(3,2);
Dth_seq0 = 1:((n_discrete(4)-1)/2);
Dth_seq1 = real(Dth_seq0.^1.2);
Dth_seq = [-Dth_seq1(end:-1:1),0,Dth_seq1]/max(Dth_seq1)*range(4,2);

par_mat0 = zeros(n_discrete([1 2 4]));

y_num = length(y_seq);
a6_cell = cell(y_num,1);
b6_cell = a6_cell;
cost_cell = a6_cell;
costCompare_cell = a6_cell;

% 初始化一个求解器
optimizer0 = TrajOptimizer_MinLTR('ltr_zeta',0.4,'ltr_omega_n',1.8);

tic
% 计算查找表的值，这里假设关键参数是某种函数，例如：
parfor i = 1:y_num
    optimizer = optimizer0;
    y = y_seq(i);
    a6_mat0 = par_mat0;
    b6_mat0 = par_mat0;
    cost_mat0 = par_mat0;
    costCompare_mat0 = par_mat0;

    for j = 1:length(x_seq)
        x = x_seq(j);
        for k = 1:length(v0_seq)
            v0 = v0_seq(k);
            for n = 1:length(Dth_seq)
                Dth = Dth_seq(n);
                v = getProximatedSpd(v0,x,y,Dth,tf);
                
                startBdry = TrajBoundaryCondition('pos',[0,0],'heading',0,'spd',v0,'acc',0);
                finalBdry = TrajBoundaryCondition('pos',[x,y],'heading',Dth,'spd',v,'acc',0);
                [~,cost,~,optParams,~,cost_compare] = optimizer.lctrajopt_mex2(tf,startBdry,finalBdry);
               
                a6_mat0(k,j,n) = optParams(1);
                b6_mat0(k,j,n) = optParams(2);
                cost_mat0(k,j,n) = cost;
                costCompare_mat0(k,j,n) = cost_compare;
            end
            disp(['当前优化的组合：v0=' num2str(v0*3.6) 'm/s x=' num2str(x) 'm y=' num2str(y) 'm Dth =' num2str(Dth*180/pi) 'deg'])
        end
    end
    a6_cell{i} = a6_mat0;
    b6_cell{i} = b6_mat0;
    cost_cell{i} = cost_mat0;
    costCompare_cell{i} = costCompare_mat0;
   
    disp(['**************************************************离线LTR参数a6 b6优化进程已经完成：' num2str(i/y_num*100) '%'])
end

% 进行整理
a6_mat = zeros(n_discrete);
b6_mat = zeros(n_discrete);
cost_mat = zeros(n_discrete);
costCompare_mat = zeros(n_discrete);

for i = 1:length(y_seq)
    y = y_seq(i);
    a6_mat0 = a6_cell{i};
    b6_mat0 = b6_cell{i};
    cost_mat0 = cost_cell{i};
    costCompare_mat0 = costCompare_cell{i};

    for j = 1:length(x_seq)
        for k = 1:length(v0_seq)
            for n = 1:length(Dth_seq)
                a6_mat(k,j,i,n) = a6_mat0(k,j,n);
                b6_mat(k,j,i,n) = b6_mat0(k,j,n);
                cost_mat(k,j,i,n) = b6_mat0(k,j,n);
                costCompare_mat(k,j,i,n) = costCompare_mat0(k,j,n);
            end
        end
    end
    disp(['******************************离线LTR参数a6 b6整理进程已经完成：' num2str(i/length(y_seq)*100) '%'])
end
% F 是一个5维数组，表示每个状态量组合对应的函数值（查找表数据）
% 需要插值的状态量组合
x_query = [32, 55, 0.5, 15/180*pi];  % 假设这些是需要查询的状态量
timeSpend = toc;
disp(['所有工况一共' num2str(prod(n_discrete)) '组，共花费时间' num2str(toc/3600) ...
    'h(' num2str(toc/60) 'min),平均每个优化花费：'  num2str(toc/prod(n_discrete)*1000) 'ms'])
tic
testNum = 1000;
for i = 1:testNum
% 使用 interpn 函数进行多维插值
f_query = interpn(v0_seq, x_seq, y_seq, Dth_seq, a6_mat, x_query(1), x_query(2), x_query(3), x_query(4), 'linear');
end
toc/testNum

% 输出查询结果
disp(['插值结果为: ', num2str(f_query)]);
%% 保存结果
save(['optimized_a6b6_' datestr(now,'yyyy-MM-dd_hh-mm') '.mat'],'a6_mat','b6_mat','cost_mat','costCompare_mat')

function v = getProximatedSpd(v0,x,y,Dth,tf)
    dist0 = sqrt(x^2+y^2);
    devTh = abs(atan2(-y,x))/pi*180; % 这是最终位置和初始速度方向的偏角
    dist_1st = dist0*interp1([0,100],[1,1.3],devTh); % 认为这个偏角越大，实际轨迹比直线连线更长
    dist_2nd = dist_1st*interp1([0,100],[1,1.3],abs(Dth)); % 又用最终方向角和初始方向角的差进行加长轨迹修正
    v = 2*(dist_2nd - v0*tf)/(tf^2);
end
