% This is a simulation scenario class for COMPASS project.
classdef SimScenario 
    properties(Constant)
        var = struct('laneIdxDev',1, ...
                       'laneDist',2, ...
                              'x',3, ...
                              'y',4, ...
                            'spd',5, ...
                            'acc',6, ...
                        'heading',7, ...
                         'edgeNo',8, ...
                       'opsState',9, ... % 0:结束运行,1:在普通边内,2:在最终边内且下一个边不在地图中啦,3:在Junction内
                 'maxLaneIdxDev',10, ...
                           'dir',11, ... % 当前正准备进行的方向(也表示了优先级)，-1,0,1,2,3(edge末端停车，调头，左转，右转，直行)
                       'remDist',12, ...
                      'routeIdx',13, ... % 在route中的idx
              'canReachNextEdge',14, ... % 当前lane是否可以到达route的下一个edge
                  'changingLane',15, ... % 是否正在换道,-1：左换道中 0:不换道 1：右换道中
                 'targetLaneIdx',16, ... % 目标的laneID
                        'spdLim',17, ... % 本edge的速度限制
                    'nextSpdLim',18, ... % 下一个edge的速度限制
                        'devSpd',19, ... % 横向偏移的速度，单位为 车道/s
                    'frontSpace',20, ... % 前方的距离
                     'backSpace',21, ... % 后方的距离
                        'desSpd',22, ... % 期望速度
                   'frontVehSpd',23);    % 前方车辆速度
        allowedTreeSet = {'R','L','RR','RL','LR','LL','RRR','RRL','RLR','RLL','LRL','LRR','LLR','LLL'};
        allowedMaxInc_L = [0   1   0    0    1    2    0     0     0     1     1     1     2     3];
        allowedMaxInc_R = [1   0   2    3    0    0    3     2     1     1     0     1     0     0];
        allowedFinalInc = [-1  1  -2    0    0    2   -3    -1    -1     1     1    -1     1     3];
    end
    properties(Hidden)
        var_laneIdxDev     = 1;
        var_laneDist       = 2;
        var_x              = 3;
        var_y              = 4;
        var_spd            = 5;
        var_acc            = 6;
        var_heading        = 7;
        var_edgeNo         = 8;
        var_opsState       = 9;
        var_maxLaneIdxDev  = 10;
        var_dir            = 11;
        var_remDist        = 12;
        var_routeIdx       = 13;
      var_canReachNextEdge = 14;
        var_changingLane   = 15;
        var_targetLaneIdx  = 16;
        var_spdLim         = 17;
        var_nextSpdLim     = 18;
        var_devSpd         = 19;
        var_frontSpace     = 20;
        var_backSpace      = 21;
        var_desSpd         = 22;
        var_frontVehSpd    = 23;
    end
    properties(Dependent)
        % 用于获得的属性
        laneIdxDev_
        laneDist_
        x_
        y_
        spd_
        acc_
        heading_
        edgeNo_
        opsState_
        maxLaneIdxDev_
        dir_
        remDist_
        routeIdx_
        canReachNextEdge_
        changingLane_
        targetLaneIdx_
        spdLim_
        nextSpdLim_
        devSpd_
        frontSpace_
        backSpace_
        desSpd_
        vehSpds_
        frontVehSpd_
        
        egoSpds_
        
    end
    properties
        % 仿真时间与步长参数
        stateNum               % 车辆的状态数量
        maxSimTime = 20;       % 最大仿真时间（所有阶段）
        timeStep = 0.5;        % 场景快速仿真用时间步长
        sectionSimTime = 2;    % 本阶段场景仿真到的时间
        currentStep = 0;       % 所有阶段目前为止累计的仿真步数  
        
        % 车辆状态意图等参数
        egoTargetLaneIdx       % 本阶段内自车的期望车道序号，默认为空
        egoTargetSpd = 60/3.6; % 本阶段内自车的期望速度，默认为60km/h
        vehState = [];         % 本阶段内所有车辆状态，第一个是自车
        vehLogState = [];      % 所有阶段所有车辆状态的记录
        vehicles = {};         % 所有车辆名义参数体，第一个是自车，不发生变化只调用
        vehDriveLine           % 所有车前方的行驶线
        vehNum = 0;            % 所有车数量，至少有1因为有自车
        surroundVeh            % 存储左前 前 右前 左后 后 右后 的周围车辆No
        surrVehDist            % 存储左前 前 右前 左后 后 右后 的周围车辆距离，都是非负的
        surrVehSpd             % 存储左前 前 右前 左后 后 右后 的周围车速度
        nextEdgeIDs            % 如果有下一个edge的话，就会存储nextEdgeID，对于状态2（自身末边）以及结束0为[]
        laneIDs                % 存储所有车当前的laneID
        edgeIDs                % 存储所有车当前的edgeID
        junctionIDs            % 存储所有车当前的junctionID
        politeness             % 存储所有车的礼貌系数

        % 本场景的路网信息 
        net                    % 本场景涉及到的路网信息
       
        % 统一的仿真参数
        foreseeTime = 5;       % 前视时间，用于控制生成的交叉口交互用的vehDriveLine的长度
        latAlertWidth = 0.60;  % 警惕自车周围左右latAlertWidth倍车道宽的周车
        defaultDist = 150;     % 前后方车辆如果没有的话，的默认距离

        % 横向动力学参数
        dev_omega = 0.9;       % 横向变化的角频率ω 1.5
        dev_zeta = 1.0;        % 横向变化的阻尼率ζ 0.8
        
        % 自车条件状态
        ego_canLC = logical([0 0]); % 自车是否能换道：左，右    
        actionTimeCounter = 0;      % 动作开始后的计数
        egoLCDuration = 4;          % 默认自车换道持续时间，必须≥此时间才能进行下次操作
        egoLC_counter = 0;          % 自车的换道计数器
        egoMaxLCNum = 3;            % 最多考虑n次自车换道
        lastRouteInfo = [];         % 上次的决策信息
        lastDecisionTimeGap = [];   % 上次决策距离本次的时间

        % 场景代价权重
        w_safe_ttc1 = 2500;         % 安全-->1/TTC 2500
        w_safe_thw = 2500;          % 安全-->前后车间距2500
        w_efficiency_spdDiff = 10;  % 高效-->实际速度和期望速度绝对差值 10
        w_smooth_acc = 100;           % 平稳-节能-->加速度大小 1 → 1000 
        w_smooth_jerk = 1;          % 平稳-节能-->jerk大小 1
        w_rule_lane = 1e4;          % 规则-->是否行驶在不期望的车道 
        w_right_lane = 5;           % 规则-->是否靠右行驶 →增加了本项
        w_consist = 10;             % 决策一致性-->上次决策和本次的差距 10
        w_envVeh = 0.3;             % 环境车的最大代价与自车代价的比例
        w_gama = 0.95;              % 代价折扣 0.95
        w_gama_prod = 1;            % 被不断折扣的代价
        w_gama_deriv = 1;           % 被不断增加的代价
        cost_compare = inf;         % 用于剪枝的对比代价，超过这个代价就被剪枝了
        

        % 额外的惩罚
        LC_penalty = 6000;            % 每次换道会额外产生的消耗  3000→8000
        % 场景限制阈值
        thrld_ttc1 = 1/2;          % 1/TTC的阈值
        thrld_thw = 1.6;           % 车头时距阈值
        

        % 场景代价累计
        cost = inf;              % 场景的累计代价
        calcCost_stepNum = 0;  % 已经计算过代价的步数

        % 场景并行算法参数
        ifSceParallel = false;     % 是否场景并行计算
        allowedPattern             % 允许的换道动作pattern
        lc1start                   % 第一次换道的最低时刻要求（包含，>=）
        lc1end                     % 第一次换道的最高时刻限度（不包含，<）
    end
    methods
        % 初始化场景
        function obj = SimScenario(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
            obj.stateNum = length(fieldnames(obj.var)); % 同步一下有多少个状态量
        end
        
        % 添加车辆，首先加入自车，随后加入他车
        function obj= addVehicle(obj,vehicle4COMPASS,vehicleState)
            obj.vehicles = [obj.vehicles;{vehicle4COMPASS}];
            obj.vehNum = obj.vehNum + 1;
            obj.vehState(obj.vehNum,:) = vehicleState;
        end
        
        % 在添加完车辆之后进行设置，初始化存储空间，之后每次仿真都只需要修改自车意图和停止时间就可以啦
        function obj = initSenario(obj) 
            obj.currentStep = 1; 
            totalStepNum = obj.maxSimTime/obj.timeStep+1;
            obj.vehLogState = zeros(totalStepNum,obj.stateNum,obj.vehNum);
            obj.laneIDs = obj.getAllLaneIDs(); % 一定要先更新这个表格，后面才能用的
            obj.edgeIDs = obj.getAllEdgeIDs();
            obj.junctionIDs = obj.getAllJunctionIDs();
            obj.politeness = obj.getAllPolitenss();

            obj.vehState(:,obj.var_canReachNextEdge) = obj.getAllCanReachNextEdge(); 
            obj.vehState(:,obj.var_remDist) = obj.getAllRemDist(); % 更新剩余距离
            obj.vehState(:,obj.var_dir) = obj.getAllDirs(); % 更新所有车下一个路口/正在的方向意图
            obj.vehDriveLine = obj.getAllVehicleLines(); % 用了canReachNextEdge,所以一定在其后更新
            obj.nextEdgeIDs = obj.getAllNextEdgeID(); % 获取下一个edge（如果有的话，对应状态1,3）
            [obj.vehState(:,obj.var_spdLim),obj.vehState(:,obj.var_nextSpdLim)] = obj.getAllSpdLim();
            [obj.surroundVeh,obj.surrVehDist,obj.surrVehSpd] = getAllSurroundVeh(obj);
            [obj.vehState(:,obj.var_frontSpace),obj.vehState(:,obj.var_backSpace), ...
                obj.vehState(:,obj.var_frontVehSpd)] = obj.getAllSpace();
            obj.vehState(:,obj.var_desSpd) = obj.getAllSpdDes();
            obj.vehState(:,obj.var_devSpd) = obj.targetLaneIdx_*2*obj.dev_zeta*obj.dev_omega;
            obj.ego_canLC = obj.getEgoCanLC;
            obj.cost = 0;
        
            obj = obj.saveState();
        end
       
        %#################################################################################################
        %##############################################关键函数############################################
        %#################################################################################################
        % 转换其他车辆状态量
        obj = convertVehState(obj,vehicleDummy);

        % 进行一步仿真
        obj = step(obj);

        % 进行换道操作
        [changingLanes, targetLaneIdxs,accs] = globalMOBIL(obj);

        % 查找每辆车的周车并更新列表
        [surroundVehMat,surrVehDistMat,surrVehSpdMat] = getAllSurroundVeh(obj);
        
        % 自车的期望车速
        egoDesSpd = getEgoDesSpd(obj)


        function obj = setAllowedPattern(obj,patternCode)
            obj.ifSceParallel = true; % 进行场景并行
            % 使用正则表达式提取信息
            tokens = regexp(patternCode,'(\w+)_a(\d{2})b(\d{2})', 'tokens');
            obj.allowedPattern = tokens{1}{1};
            obj.lc1start = str2double(tokens{1}{2});
            obj.lc1end = str2double(tokens{1}{3});
            obj.egoTargetLaneIdx = obj.vehState(1,obj.var_targetLaneIdx);
        end

        function patternCode = showAllowedPattern(obj)
            if ~isempty(obj.allowedPattern)
                if obj.lc1start < 10
                    lc1start_char = ['0' num2str(obj.lc1start)];
                else
                    lc1start_char = num2str(obj.lc1start);
                end
                if obj.lc1end < 10
                    lc1end_char = ['0' num2str(obj.lc1end)];
                else
                    lc1end_char = num2str(obj.lc1end);
                end
                patternCode = [obj.allowedPattern '_a' lc1start_char 'b' lc1end_char];
            else
                patternCode = [];
                disp('没有设置当前场景的Pattern喔，输出[]')
            end
        end
        % 根据交通规则获得剔除不合规的场景树
        function trimmedTreeSet = getTrimmedTreeSet(obj)
            % 获取自车必要数据
            egoState = obj.vehState(1,:);
            egoSpd = egoState(obj.var_spd);
            remDist = egoState(obj.var_remDist);
            totSimTime = obj.maxSimTime;
            allowedDecel = obj.vehicles{1}.b_comf; % 使用舒适减速度验证
            theTime = min(totSimTime,egoSpd/allowedDecel);
            minDist = egoSpd*theTime - 1/2*allowedDecel*theTime^2;
            maxLaneIdx = egoState(obj.var_maxLaneIdxDev)-0.5;
            laneIdx = max(min(round(egoState(obj.var_laneIdxDev)),maxLaneIdx),0);
            distLim = max(remDist - 50,0); % 不能小于0啦
            distAllowedIdxs = 1:14;
            if minDist > distLim % 如果减速也不行，那就按照交规来
                desLaneIdxSet = obj.getDesLaneIdxSet();
                twoLC_time = 10;
                twoLC_time = min(theTime,twoLC_time);
                if (egoSpd*twoLC_time - 1/2*allowedDecel*twoLC_time^2)< distLim
                    % distAllowedIdxs(7:end) = 0; % 只允许2次换道
                else
                    % distAllowedIdxs(3:end) = 0; % 只允许1次换道
                end
                
            else % 距离路口还是比较远，那我就随意一些开车啦，所有车道都可以通行
                desLaneIdxSet = 0:maxLaneIdx;
            end
            maxInc_L = maxLaneIdx - laneIdx;
            maxInc_R = laneIdx;
            allowedFinalInc0 = desLaneIdxSet - laneIdx;
            final_ok = ismember(obj.allowedFinalInc,allowedFinalInc0);
            left_ok = obj.allowedMaxInc_L <= maxInc_L;
            right_ok = obj.allowedMaxInc_R <= maxInc_R;
            if sum(final_ok) == 1
                trimmedIdxs = final_ok;
            else
                trimmedIdxs = left_ok & right_ok & final_ok & distAllowedIdxs;
            end
            if sum(trimmedIdxs) == 0 % 如果出现了不包含直行（L,R）的情况，那至少车道保持是可以的啦
                trimmedIdxs(1:2) = 1;
            end
            trimmedTreeSet = obj.allowedTreeSet(trimmedIdxs);
        end
        

        function flag = isEgoInJunction(obj) % 判断自车是否在交叉口内，如果是，就沿着当前车道驶出路口再COMPASS
            flag = false;
            if obj.vehState(1,obj.var_opsState) ~= 3
                flag = true;
            end
        end
        
        % 当自车在非交叉路口、非最后一段路时，根据交通规则获取自车在
        function egoDesLaneIdxSet = getDesLaneIdxSet(obj)
            egoState = obj.vehState(1,:);
            opsState = egoState(obj.var_opsState);
            maxLaneIdx = egoState(obj.var_maxLaneIdxDev)-0.5;
            if opsState == 1 || opsState == 2 % 如果在正常边内，就是有下一个边
                edgeID = obj.edgeIDs{1};
                nextEdgeID = obj.vehicles{1}.route{obj.routeIdx_(1)+1};
                candidateLaneIdx = 0:maxLaneIdx;
                candidateIdx = false(1,maxLaneIdx+1);
                for i = 1:maxLaneIdx+1
                    if obj.net.canReach_L2E([edgeID '_' char(48+i-1)],nextEdgeID)
                        candidateIdx(i) = true;
                    end
                end
                egoDesLaneIdxSet = candidateLaneIdx(candidateIdx);
                % dir = egoState(obj.var_dir); % 自车方向
                % -1,0,1,2,3(edge末端停车，调头，左转，右转，直行)
                % switch dir
                %     case {-1,2} % 末端停车、右转，靠右行
                %         egoDesLaneIdxSet = 0;
                %     case {0,1} % 调头、左转，靠左行
                %         egoDesLaneIdxSet = maxLaneIdx;
                %     otherwise % 直行
                %         egoDesLaneIdxSet = 0:maxLaneIdx;
                % end
            else % 自车应该不会结束哒，到这里应该是在边边内部。
                laneIdx = max(min(round(egoState(obj.var_laneIdxDev)),maxLaneIdx),0);
                egoDesLaneIdxSet = laneIdx; % 沿着当前车道就行啦
            end
        end
        
        function [TTC_ok, THW_ok] = getEgoTTC_THW(obj) % 获得自车左右侧前后车辆的最小TTC，THW
            THW_thred = obj.vehicles{1}.T_hw;
            % THW_thred = obj.thrld_thw;
            TTC_thred = 1/obj.thrld_ttc1;
            s_min = obj.vehicles{1}.s_min;
            egoSpd = obj.spd_(1);
            distMat = obj.surrVehDist(1:2,[1 3]) - obj.vehicles{1}.L; % →/2 #要修改
            TTC = distMat./(obj.surrVehSpd(1:2,[1 3]) - egoSpd);
            TTC(TTC < 0) = 100;
            TTC_ok = min(TTC) >= TTC_thred;

            THW = (obj.surrVehDist(1:2,[1 3]) - s_min)./egoSpd;
            THW_ok = min(THW) > THW_thred;

        end
        %#################################################################################################
        %#################################################################################################
        %#################################################################################################
        % 检查得到自车是否可以换道
        function ego_canLC = getEgoCanLC(obj)
            currentTime = obj.getSimTime;
            ego_canLC = (obj.surroundVeh(1,[1 3]) ~= 999) & ~obj.actionTimeCounter;
            LC_counter = obj.egoLC_counter;
            % 如果到最大仿真时间之前的时间小于单次换道允许的时间，那么就是不能换道
            if (currentTime + obj.egoLCDuration) > obj.maxSimTime
                ego_canLC = logical([0 0]);

            elseif LC_counter >= obj.egoMaxLCNum % 如果已经换道了这么多次啦
                ego_canLC = logical([0 0]);

            elseif obj.remDist_(1) < 50 % 如果到了路口前50m了，也就不能换道啦
                ego_canLC = logical([0 0]);

            elseif obj.opsState_(1) == 3
                ego_canLC = logical([0 0]);
            end
            % 如果是左右侧前后车辆的TTC过小，或者THW过小都不能换道
            [TTC_ok, THW_ok] = obj.getEgoTTC_THW();
            ego_canLC = ego_canLC & TTC_ok & THW_ok;
            
            if obj.ifSceParallel % 如果是场景并行计算，只需要考虑这个场景的可行换道序列
                if isempty(obj.allowedPattern)
                    error('先在设置为场景并行模式，请设置obj.allowedPattern哦')
                else
                    if LC_counter >= length(obj.allowedPattern) % 如果已经换道了这么多次啦
                        ego_canLC = logical([0 0]);   
                    else
                        if LC_counter == 0
                            if (currentTime < obj.lc1end) && (currentTime >= obj.lc1start)
                                treeAllow = logical([1 1]);
                            else
                                treeAllow = logical([0 0]);
                            end
                        else
                            treeAllow = logical([1 1]);
                        end
                        switch obj.allowedPattern(LC_counter+1)
                            case 'L'
                                LC_lim = logical([1 0]);
                            case 'R'
                                LC_lim = logical([0 1]);
                        end
                        ego_canLC = ego_canLC & LC_lim & treeAllow;
                    end
                end
            end
        end

        function obj = setAction(obj,actionName)
            if isempty(obj.egoTargetLaneIdx)
                obj.egoTargetLaneIdx = obj.targetLaneIdx_(1);
            end
            switch actionName
                case 'L'
                    inc_egoTargetLaneIdx = 1;
                    obj.egoLC_counter = obj.egoLC_counter + 1;
                    obj.vehState(1,obj.var_changingLane) = obj.egoLC_counter;
                    obj.cost = obj.cost + obj.LC_penalty; % 变到内部啦→
                case 'R'
                    inc_egoTargetLaneIdx = -1;
                    obj.egoLC_counter = obj.egoLC_counter + 1;
                    obj.vehState(1,obj.var_changingLane) = -obj.egoLC_counter;
                    obj.cost = obj.cost + obj.LC_penalty;
                case 'K'
                    inc_egoTargetLaneIdx = 0;
                    obj.vehState(1,obj.var_changingLane) = 0;
                otherwise
                    error('未定义的动作哦，只能是LKR其中之一');
            end 
            obj.egoTargetLaneIdx = obj.egoTargetLaneIdx + inc_egoTargetLaneIdx;
            obj.actionTimeCounter = obj.egoLCDuration;
            
            
        end
        
        %#################################################################################################
        %#########################################剪枝算法#################################################
        %#################################################################################################
        function flag = trim(obj) % 剪枝算法，看是否需要把这个场景减掉
            flag = false;

            % 后续继续补充其他剪枝条件
            if obj.ifSceParallel % 如果是场景并行计算
                if obj.cost > obj.cost_compare
                    flag = true;
                    disp(['--------场景' obj.allowedPattern '由于由于代价超过对比值被剪枝，' ...
                        '当前已经完成' char(48+obj.egoLC_counter) '次换道'])
            % 如果距离路口到达给定距离，如果还没到规定车道，就要剪掉啦
                elseif obj.remDist_(1) < 30 && ~obj.canReachNextEdge_(1)
                    flag = true;
                    disp(['--------场景' obj.allowedPattern '由于在路口30米内还没到规定车道被剪枝，' ...
                        '当前已经完成' char(48+obj.egoLC_counter) '次换道'])
                elseif length(obj.allowedPattern) ~= 1 % 如果只进行一次换道，就不考虑对不换道这种
                    % 如果剩余时间不足以使其完成剩余的pattern操作，就剪枝掉
                    remainActionTime = obj.actionTimeCounter + (length(obj.allowedPattern) - obj.egoLC_counter)*obj.egoLCDuration;
                    if (obj.maxSimTime - obj.getSimTime) < remainActionTime
                        flag = true;
                        % if strcmp(obj.allowedPattern,'L')
                        disp(['--------场景' obj.allowedPattern '由于在剩余时间不够完成动作被剪枝，' ...
                            '当前已经完成' char(48+obj.egoLC_counter) '次换道'])
                        % end
                    end
                end
            end
        end

        %#################################################################################################
        %#################################################################################################
        %#################################################################################################


        % 获取所有车的期望车速(最高限速)
        function spdDesList = getAllSpdDes(obj) % 需要提前更新好remDist
            spdDesList = zeros(obj.vehNum,1);
            remDists = obj.vehState(:,obj.var_remDist);
            spdLims = obj.vehState(:,obj.var_spdLim);
            nextSpdLims = obj.vehState(:,obj.var_nextSpdLim);
            for vNo = 1:obj.vehNum
                spdLim = spdLims(vNo);
                nextSpdLim = nextSpdLims(vNo);
                if spdLim <= nextSpdLim % 如果当前车道限速较低，就继续按照这个跑
                    spdDesList(vNo) = spdLim;
                else % 如果当前车道限速较高，就按照本车的舒适减速度来计算，看期望车速是多少
                    b_comf = obj.vehicles{vNo}.b_comf/2; %# 要修改，*10了，为了模拟路口减速不及时的实际情况
                    if vNo > 1
                        b_comf = b_comf*10; % #要修改
                    end
                    s_realm = abs(spdLim*spdLim - nextSpdLim*nextSpdLim)/(2*b_comf);
                    if remDists(vNo) > s_realm % 还没有到影响范围内，也就是交叉口减速区
                        spdDesList(vNo) = spdLim;
                    else
                        spdDesList(vNo) = sqrt(2*remDists(vNo)*b_comf + nextSpdLim*nextSpdLim);
                    end

                end
            end
            % spdDesList(1) = abs(min(spdDesList(1),obj.getEgoDesSpd())); %#要修改
            spdDesList(1) = obj.getEgoDesSpd();
            % spdDesList(2) = 1/3.6; % #要修改，我自己随意为了测试自车行为加入的
        end


        % 一次性获取所有的当前edge速度限制和下一个edge的速度限制
        function [spdLims,nextSpdLims] = getAllSpdLim(obj) % 需要事先初始化好dirs，nextEdgeIDs
            spdLims = zeros(obj.vehNum,1);
            nextSpdLims = spdLims;
            opsStates = obj.opsState_;
            dirs = obj.dir_;
            for vNo = 1:obj.vehNum
                if opsStates(vNo) % 如果车辆还没结束，就更新一下
                    spdLims(vNo) = obj.net.e_dict{obj.laneIDs{vNo}}.speed;
                    nextEdgeID = obj.nextEdgeIDs{vNo};
                    if isempty(nextEdgeID) || ~obj.net.isInNet_fast(nextEdgeID)
                        switch dirs(vNo)
                            case -1 % edge末停车
                                nextSpdLims(vNo) = 0;
                            case  0 % 调头
                                nextSpdLims(vNo) = 5/3.6;
                            case  1 % 左转
                                nextSpdLims(vNo) = 30/3.6;
                            case  2 % 右转
                                nextSpdLims(vNo) = 30/3.6;
                            case  3 % 直行
                                nextSpdLims(vNo) = spdLims(vNo);
                            otherwise
                                nextSpdLims(vNo) = 0;
                        end
                    else
                        nextSpdLims(vNo) = obj.net.e_dict{[nextEdgeID '_0']}.speed;
                    end
                end
            end
        end
        

        % #############################################################################################################
        %                                           各属性的一次性全更新
        % #############################################################################################################

        %--------------------------------------------------------------------------------------------------------------
        %-------------------------------------------属性：opsState--------------------全部更新--------------------------
        %-------------------------------------------------------------------------------------------------------------
        % 获取所有车的opsState用于更新
        function opsStateList = getAllOpsState(obj)
            opsStateList = obj.vehState(:,obj.var_opsState);
            for vNo = 1:obj.vehNum
                if opsStateList(vNo) > 0 % 只更新没结束的车辆
                    edgeNo = obj.vehState(vNo,obj.var_edgeNo);
                    isInFinalEdge = obj.net.frindgeEdgeArray(edgeNo);
                    routeIdx = obj.vehState(vNo,obj.var_routeIdx);
                    isInEndEdge = (routeIdx >= obj.vehicles{vNo}.routeNum); % 看下是不是到估计的路线的最终边啦
                    if ~isInEndEdge
                        % 如果不是，再看看下一条边是不是在本场景的路网外部
                        isNextEdgeOutRange = ~obj.net.isInNet_fast(obj.vehicles{vNo}.route{routeIdx+1});
                    end
                    if strncmp(obj.laneIDs{vNo},':',1)
                        opsState = 3;
                    elseif isInEndEdge || (isInFinalEdge && isNextEdgeOutRange) 
                        % 2 的含义就是，要么在这辆车自己的结束，要么因为路网裁剪的原因被迫结束
                        opsState = 2;
                    else 
                        opsState = 1;
                    end
                    opsStateList(vNo) = opsState;

                end
            end
            
        end

        %--------------------------------------------------------------------------------------------------------------
        %----------------------------属性：canReachNextEdge--------------------全部更新-------------------------------
        %-------------------------------------------------------------------------------------------------------------
        % 判断所有车辆是否可以到达其路线上的下一个edge
        function canReachNextEdgeList = getAllCanReachNextEdge(obj)
            canReachNextEdgeList = zeros(obj.vehNum,1); % 全都初始化为不能
            for vNo = 1:obj.vehNum
                if obj.vehState(vNo,obj.var_opsState) % 如果还没结束，就计算，否则就不浪费算力啦
                    % 对每辆车判断是否可以到达下一个边
                    currentLaneID = obj.laneIDs{vNo};
                    nextEdgeID = obj.getNextRouteEdgeID(vNo);
                    if ~isempty(nextEdgeID) % 如果已经没有下一个edge了，就不用管啦，否则继续
                        canReachNextEdgeList(vNo) = obj.net.canReach_L2E(currentLaneID,nextEdgeID);
                    end
                end
            end
        end


        %--------------------------------------------------------------------------------------------------------------
        %-------------------------------------------属性：dir--------------------全部更新-------------------------------
        %-------------------------------------------------------------------------------------------------------------
        % 查找每一辆车在下一个路口时的意图的语义
        function dirs = getAllDirs(obj) % 当前正准备进行的方向(也表示了优先级)，-1,0,1,2,3(edge末端停车，调头，左转，右转，直行)
            dirs = -1*ones(obj.vehNum,1); % 先用-1初始化
            for vNo = 1:obj.vehNum
                opsState = obj.vehState(vNo,obj.var_opsState);
                if opsState % 如果还没结束,就继续判断
                    routeIdx = obj.vehState(vNo,obj.var_routeIdx);
                    routeNum = obj.vehicles{vNo}.routeNum;
                    currentEdgeID = obj.edgeIDs{vNo};
                    if routeIdx < routeNum % 如果没有后续啦，认为车将在本edge末停车,否则↓%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%可以认真修改的地方
                        % 如果是在交叉口内，说明一定还有下一个edge
                        % 在正常边内，并不是要结束了，还有下一个边
                        nextEdgeID = obj.getNextRouteEdgeID(vNo); % 获取route中的下一个edge
                        dir = obj.net.getDirE2E(currentEdgeID,nextEdgeID);
                        if strcmpi(dir,'s')
                            dirs(vNo) = 3;
                        elseif strcmpi(dir,'r')
                            dirs(vNo) = 2;
                        elseif strcmpi(dir,'l')
                            dirs(vNo) = 1;
                        elseif strcmpi(dir,'t')
                            dirs(vNo) = 0;
                        end
                    end
                end
            end
        end
        %--------------------------------------------------------------------------------------------------------------
        %-------------------------------------------属性：remDist-----------------------全部更新------------------------
        %-------------------------------------------------------------------------------------------------------------
        % 一次性更新所有的remDist，适合初始化时使用
        function  remDistList = getAllRemDist(obj)
            remDistList = zeros(obj.vehNum,1);
            opsState = obj.opsState_;
            for vNo = 1:obj.vehNum
                if opsState(vNo)
                    currentLaneID = obj.laneIDs{vNo};
                    remDistList(vNo) = obj.net.e_dict{currentLaneID}.length;
                end
            end
            remDistList = remDistList - obj.vehState(:,obj.var_laneDist);
        end

        %--------------------------------------------------------------------------------------------------------------
        %------------------------------------------场景状态：vehDriveLine------------------全部更新----------------------
        %-------------------------------------------------------------------------------------------------------------
        % 对每辆车，找出每一辆车的当前车道下向前foreseeTime秒匀速行驶距离的车道中心线
        vehDriveLine_init = getAllVehicleLines(obj)
 
        function politenss = getAllPolitenss(obj)
            vehNumber = obj.vehNum;
            politenss = zeros(vehNumber,1);
            for vNo = 1:vehNumber
                politenss(vNo) = obj.vehicles{vNo}.politenss;
            end
        end

        %  一次性不加判断地获取所有车辆的junctionID,只在初始化时推荐使用一次.
        function junctionIDs = getAllJunctionIDs(obj)
            opsState = obj.opsState_;
            junctionIDs = cell(obj.vehNum,1);
            for vNo = 1:obj.vehNum
                if opsState(vNo)
                    edgeID = obj.edgeIDs{vNo};
                    if strncmp(edgeID,':',1) %说明是Junction的内部edge
                        lastUnderscoreIdx = find(edgeID=='_',1,'last');
                        junctionIDs{vNo} = edgeID(2:lastUnderscoreIdx-1);
                    else
                        junctionIDs{vNo} = obj.net.e_dict{edgeID}.to;
                    end
                end
            end
        end

        % 一次性不加判断地获取所有车辆的laneID或edgeID,只在初始化时推荐使用一次。
        function edgeIDs = getAllEdgeIDs(obj)
            opsState = obj.opsState_;
            edgeIDs = cell(obj.vehNum,1);
            for vNo = 1:obj.vehNum
                if opsState(vNo)
                    edgeIDs{vNo} = obj.getEdgeID(vNo);
                end
            end
        end
        function laneIDs = getAllLaneIDs(obj)
            laneIDs = cell(obj.vehNum,1);
            opsStates = obj.opsState_;
            for vNo = 1:obj.vehNum
                if opsStates(vNo)
                    laneIDs{vNo} = obj.getLaneID(vNo);
                end
            end
        end
    


        % 获取车辆当前的laneID
        function laneID = getLaneID(obj,vNo)
            maxLaneIdx = round(obj.vehState(vNo,obj.var_maxLaneIdxDev)-0.5);
            laneIdx = round(obj.vehState(vNo,obj.var_laneIdxDev));
            if laneIdx > maxLaneIdx
                laneIdx = maxLaneIdx;
            elseif laneIdx < 0
                laneIdx = 0;
            end
            laneID = [obj.net.edgeList{obj.vehState(vNo,obj.var_edgeNo)} '_' char('0'+laneIdx)];
        end

        % 获取车辆当前的edgeID
        function edgeID = getEdgeID(obj,vNo)
            edgeID = obj.net.edgeList{obj.vehState(vNo,obj.var_edgeNo)};
        end
        
        function nextEdgeIDs = getAllNextEdgeID(obj)
            vehNumber = obj.vehNum;
            nextEdgeIDs = cell(vehNumber,1);
            opsStates = obj.opsState_;
            for vNo = 1:vehNumber
                if opsStates(vNo) % 如果车辆没结束，就找一下
                    nextEdgeIDs{vNo} = obj.getNextEdgeID(vNo);
                end
            end
        end
        
        function [frontSpaces,backSpaces,frontVehSpds] = getAllSpace(obj)
            frontSpaces = obj.surrVehDist(1:2:end,2);
            backSpaces = obj.surrVehDist(2:2:end,2);
            frontVehSpds = obj.surrVehSpd(1:2:end,2);
        end


        % 按照路径，找到下一个edgeID,如果是快要结束了，就是空的[]
        function nextEdgeID = getNextEdgeID(obj,vNo)
            nextEdgeID = [];
            currentEdgeID = obj.edgeIDs{vNo};
            if strncmp(currentEdgeID,':',1) % 如果是在交叉口内部的edge,就直接找下一个to的edge
                nextEdgeID = obj.getNextRouteEdgeID(vNo);
            else % 如果是正常的edge，就需要从所有的connection中判断，找到第一个到这条edge的连接，取via的lane，取其edge
                opsState = obj.vehState(vNo,obj.var_opsState);
                if opsState && opsState ~= 2 % 保证了有下一个routeEdge
                    nextRouteEdgeID = obj.getNextRouteEdgeID(vNo);
                    connections = obj.net.c_from_dict{currentEdgeID}.connections;
                    for i = 1:obj.net.c_from_dict{currentEdgeID}.connection_num
                        if strcmp(connections{i}.to,nextRouteEdgeID)
                            viaLaneID = connections{i}.via;
                            nextEdgeID = regexprep(viaLaneID, '_\d+$', '');
                            break
                        end
                    end
                end
            end

        end


        % 获取route中下一个edge的ID，如果没有下一个edge，就返回[]
        function edgeID = getNextRouteEdgeID(obj,vNo)
            routeIdx = obj.vehState(vNo,obj.var_routeIdx);
            routeNum = obj.vehicles{vNo}.routeNum;
            if routeIdx >= routeNum
                edgeID = [];
            else
                edgeID = obj.vehicles{vNo}.route{routeIdx+1};
            end
        end
        
        function entity = getEntity(obj,entityID)
            entity = obj.net.getEntity(entityID);
        end

        % 获取当前仿真到的时间
        function currentTime = getSimTime(obj)
            currentTime = max((obj.currentStep-1)*obj.timeStep,0);
        end

        % 保存当前的vehState到vehLogState中去
        function obj = saveState(obj)
            obj.vehLogState(obj.currentStep,:,:) = permute(obj.vehState, [2, 1]);
        end
        
        % 根据仿真时间判断是否结束仿真
        function flag = isEnd(obj) 
            if obj.getSimTime >= obj.sectionSimTime
                flag = true;
            else
                flag = false;
            end
        end
        
        % 进行仿真，将仿真时间拉到设定的本小节终止时间，并计算这期间的代价累加到原有代价上
        function obj = simulate(obj) 
            while ~obj.isEnd
                obj = obj.step;
            end
            if obj.calcCost_stepNum < obj.currentStep
                obj = obj.calcCost;
            end
            obj.ego_canLC = obj.getEgoCanLC;
        end
        
        function obj = calcCost(obj) % 计算新的一小节这期间的代价并累加到原代价上
            vehNumber = obj.vehNum;
            weight = obj.w_envVeh;
            vehLogStates = obj.vehLogState(obj.calcCost_stepNum+1:obj.currentStep,:,:);
            veh_spds = reshape(vehLogStates(:,obj.var_spd,:),[],vehNumber);
            % des_spds = reshape(vehLogStates(:,obj.var_desSpd,:),[],vehNumber);
            veh_frontSpace = reshape(vehLogStates(:,obj.var_frontSpace,:),[],vehNumber);
            veh_frontVehSpd = reshape(vehLogStates(:,obj.var_frontVehSpd,:),[],vehNumber);
            ego_x = vehLogStates(:,obj.var_x,1);
            ego_y = vehLogStates(:,obj.var_y,1);
            ego_remDists = vehLogStates(:,obj.var_remDist,1);
            ego_dirs = vehLogStates(:,obj.var_dir,1);

            % 安全性---1/TTC、前车间距
            TTC_1 = max((veh_spds-veh_frontVehSpd)./veh_frontSpace,0);
            TTC_1_penalty = sum(1./(1+exp(10*(obj.thrld_ttc1 - TTC_1))));
            THW_penalty = sum(1./(1+exp(-10*(obj.thrld_thw - veh_frontSpace./veh_spds))));
            j_safe = obj.w_safe_ttc1*calcWeightedCost(TTC_1_penalty,weight,vehNumber)+ ...
                     obj.w_safe_thw*calcWeightedCost(THW_penalty,weight,vehNumber);
            % 高效性---和期望速度的绝对差值
            SPD_DIFF_penalty = sum(max(mean(veh_spds + 5) - veh_spds,0).^2);
            j_effi = obj.w_efficiency_spdDiff*calcWeightedCost(SPD_DIFF_penalty,weight,vehNumber);%%%%%%%%%%%%%%%%→
            % SPD_DIFF_penalty = sum(max(des_spds - veh_spds,0).^2);
            % j_effi = obj.w_efficiency_spdDiff*calcWeightedCost(SPD_DIFF_penalty,weight,vehNumber);
            % 平稳-经济性---加速度、加加速度
            ego_acc = vehLogStates(:,obj.var_acc,1);
                
            j_smooth = obj.w_smooth_acc*sum((ego_dirs>=3).*(min(ego_acc,0).^2)); % → 修改成了讨厌减速的
            
            % 规则---和期望车道的偏离、和最右车道的偏离
            ego_laneIdxDev = vehLogStates(:,obj.var_laneIdxDev,1);
            egoDesLaneIdxSet = obj.getDesLaneIdxSet;
            j_rule = obj.w_rule_lane*sum((~sum(round(ego_laneIdxDev)==egoDesLaneIdxSet,2).*max(1-ego_remDists/500,0)));
            j_rule = j_rule + obj.w_right_lane*sum((ego_dirs>=3).*round(ego_laneIdxDev));% → 增加了靠右行驶项
            % 决策一致性--和上一次决策的偏差尽量小
            lastRoute = obj.lastRouteInfo;
            if ~isempty(lastRoute) % 如果有上次的记录
                % 最大实际上就到20s
                maxSimStep = length(lastRoute);
                passStep = round(obj.lastDecisionTimeGap/obj.timeStep);
                pointNum = length(ego_x);
                calcStartStep = (obj.calcCost_stepNum+1+passStep);
                if  calcStartStep > maxSimStep
                    j_consist = 0;
                else
                    calcLen = min(pointNum,maxSimStep - calcStartStep + 1);
                    lastRoute_x = lastRoute(calcStartStep:calcStartStep+calcLen-1,1);
                    lastRoute_y = lastRoute(calcStartStep:calcStartStep+calcLen-1,2);
                    j_consist = obj.w_consist*sum(sqrt((lastRoute_x-ego_x(1:calcLen)).^2 ...
                                +(lastRoute_y-ego_y(1:calcLen)).^2));
                    % disp(['j_consist=' num2str(j_consist)])
                end
            else
                j_consist = 0;
            end
            obj.w_gama_prod = obj.w_gama*obj.w_gama_prod; % 用于随着时间不断衰减的约束
            obj.w_gama_deriv = obj.w_gama_deriv/obj.w_gama; % 用于随着时间不断增加的约束
            obj.cost = obj.cost + j_smooth + obj.w_gama_prod*(j_consist+j_safe) + obj.w_gama_deriv*(j_rule+j_effi);
            obj.calcCost_stepNum = obj.currentStep;
            function weightedCost = calcWeightedCost(cost,weight,vehNumber)
                if vehNumber > 1
                    weightedCost = cost(1) + weight*mean(cost(2:end));
                else
                    weightedCost = cost;
                end
            end
        end

        % 检查一辆车是否已经终止运行
        function flag = isDone(obj,vehNo)
            if obj.vehState(vehNo,obj.var_opsState)
                flag = true;
            else
                flag = false;
            end
        end

        % #############################################################################################################
        %                                           车辆属性get函数
        % #############################################################################################################
        function laneIdxDevs = get.laneIdxDev_(obj)
            laneIdxDevs = obj.vehState(:,obj.var_laneIdxDev);
        end
        function laneDists = get.laneDist_(obj)
            laneDists = obj.vehState(:,obj.var_laneDist);
        end
        function xs = get.x_(obj)
            xs = obj.vehState(:,obj.var_x);
        end
        function ys = get.y_(obj)
            ys = obj.vehState(:,obj.var_y);
        end
        function spds = get.spd_(obj)
            spds = obj.vehState(:,obj.var_spd);
        end
        function accs = get.acc_(obj)
            accs = obj.vehState(:,obj.var_acc);
        end
        function headings = get.heading_(obj)
            headings = obj.vehState(:,obj.var_heading);
        end
        function edgeNos = get.edgeNo_(obj)
            edgeNos = obj.vehState(:,obj.var_edgeNo);
        end
        function opsStates = get.opsState_(obj)
            opsStates = obj.vehState(:,obj.var_opsState);
        end
        function maxLaneIdxDevs = get.maxLaneIdxDev_(obj)
            maxLaneIdxDevs = obj.vehState(:,obj.var_maxLaneIdxDev);
        end
        function dirs = get.dir_(obj)
            dirs = obj.vehState(:,obj.var_dir);
        end
        function remDists = get.remDist_(obj)
            remDists = obj.vehState(:,obj.var_remDist);
        end
        function routeIdxs = get.routeIdx_(obj)
            routeIdxs = obj.vehState(:,obj.var_routeIdx);
        end
        function canReachNextEdges = get.canReachNextEdge_(obj)
            canReachNextEdges = obj.vehState(:,obj.var_canReachNextEdge);
        end
        function changingLanes = get.changingLane_(obj)
            changingLanes = obj.vehState(:,obj.var_changingLane);
        end
        function targetLaneIdxs = get.targetLaneIdx_(obj)
            targetLaneIdxs = obj.vehState(:,obj.var_targetLaneIdx);
        end
        function spdLims = get.spdLim_(obj)
            spdLims = obj.vehState(:,obj.var_spdLim);
        end
        function nextSpdLims = get.nextSpdLim_(obj)
            nextSpdLims = obj.vehState(:,obj.var_nextSpdLim);
        end
        function devSpds = get.devSpd_(obj)
            devSpds = obj.vehState(:,obj.var_devSpd);
        end
        function frontSpaces = get.frontSpace_(obj)
            frontSpaces = obj.vehState(:,obj.var_frontSpace);
        end
        function backSpaces = get.backSpace_(obj)
            backSpaces = obj.vehState(:,obj.var_backSpace);
        end
        function desSpds = get.desSpd_(obj)
            desSpds = obj.vehState(:,obj.var_desSpd);
        end
        function frontVehSpds = get.frontVehSpd_(obj)
            frontVehSpds = obj.vehState(:,obj.var_frontVehSpd);
        end
        function vehSpds = get.vehSpds_(obj)
            vehSpds = obj.vehLogState(1:obj.currentStep,obj.var_spd,:);
        end
        function egoSpds = get.egoSpds_(obj)
            egoSpds = obj.vehLogState(1:obj.currentStep,obj.var_spd,1);
        end
        function logData = getLogData(obj,vNo,varSeqNo)
            logData = reshape(obj.vehLogState(1:obj.currentStep,varSeqNo,vNo),[],length(vNo));
        end
        function LC_decision = getDecision(obj)
            LCD_idxs = diff(obj.getLogData(1,obj.var_changingLane));
            LC_step_inc = obj.egoLCDuration/obj.timeStep;
            % 可以提前一点换道，在时间步round(LC_step_inc/2)内有换道建议，就及早执行
            LC_decision = sum(LCD_idxs(1:round(LC_step_inc/4))); 
            % if ~
            
        end
        function trajSet = getAllLogTraj(obj,dimension)
            vehNumber = obj.vehNum;
            trajSet = cell(vehNumber,1);
            if nargin < 2 || dimension == 2
                for vNo = 1:vehNumber
                    trajSet{vNo} = [obj.getLogData(vNo,obj.var_x),...
                                    obj.getLogData(vNo,obj.var_y)];
                end
            else
                for vNo = 1:vehNumber
                    trajSet{vNo} = [obj.getLogData(vNo,obj.var_x),...
                                    obj.getLogData(vNo,obj.var_y),...
                                   linspace(0,10,obj.currentStep)'];
                end
            end
        end
        function accCmds = getAccCmds(obj,realSampleTime)
            simSampleTime = obj.timeStep;
            accCmds_raw = obj.getLogData(1,obj.var_acc);
            currentTime = (obj.currentStep-1)*simSampleTime;
            accCmds = interp1((0:simSampleTime:currentTime)',accCmds_raw, ...
                linspace(0,currentTime,round(currentTime/realSampleTime)+1));
        end
        function spdDesCmds = getSpdDesCmds(obj,realSampleTime)
            simSampleTime = obj.timeStep;
            spdDesCmds_raw = obj.getLogData(1,obj.var_spd);
            currentTime = (obj.currentStep-1)*simSampleTime;
            spdDesCmds = interp1((0:simSampleTime:currentTime)',spdDesCmds_raw, ...
                linspace(0,currentTime,round(currentTime/realSampleTime)+1));
        end
        function route = getEgoRoute(obj,dimension)
            if nargin < 2 || dimension == 2
                route = [obj.getLogData(1,obj.var_x),obj.getLogData(1,obj.var_y)];
            else
                route = [obj.getLogData(1,obj.var_x),obj.getLogData(1,obj.var_y),linspace(0,10,obj.currentStep)'];
            end
        end
        function initIdxDev = getEgoInitIdxDev(obj)
            logData = reshape(obj.vehLogState(1:obj.currentStep,obj.var_laneIdxDev,1),[],1);
            initIdxDev = mod(logData(1),1); %#注意车道宽度3.2m
            if initIdxDev > 0.5
                initIdxDev = initIdxDev - 1;
            end
        end
        function newobj = clone(obj) 
            newobj = SimScenario();
            metaobj = metaclass(obj);
            props = {metaobj.PropertyList.Name};
            for j = 1:length(props)
                theProp = props{j};
                if ~strcmp(theProp,'var') && ~strcmp(theProp(end),'_')
                    tmpProp = obj.(theProp);
                    if (isa(tmpProp,'handle')) % 如果是Handle类对象，调用该类的Clone方法
                        newobj.(theProp) = tmpProp.clone();
                    else % 否则直接赋值拷贝
                        newobj.(theProp) = obj.(theProp);
                    end
                end
            end
        end
    end



end