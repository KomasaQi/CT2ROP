%% 进行一次DynMC-SimTree搜索并获取规划路径route

thePool = gcp();
coreNum = thePool.NumWorkers;
initCurrentScenario
if ~ego.changeLane % 如果自车当前没有在换道，就可以规划哒
    [tempRoute,theTempScene,results] = callParallel_DynMC_SimTree(s,coreNum);
    if ~isempty(tempRoute)
        route = tempRoute;
        theScene = theTempScene;
        % 对results进行展示
        validResultCounter = 0;
        for resNum = 1:length(results)   
            theResult = results{resNum};
            if ~isempty(theResult)
                validResultCounter = validResultCounter + 1;
                disp(['有效场景' num2str(validResultCounter) ': ' theResult.showAllowedPattern...
                    '，代价：' num2str(theResult.cost) '，调用序号：' num2str(resNum) ])
            end
        end
        disp([repmat('-- * ',1,10) '最终选择场景' theScene.showAllowedPattern])
        if strcmp(ctrlMode,'SUMO')
            traci.vehicle.setLaneChangeMode(ego.vehID, 0b000000000000); % 把控制权给COMPASS
            traci.vehicle.setSpeedMode(ego.vehID,0b011110); % bit5 bit4 ... bit0. bit0是考虑安全速度，把其勾选走了
            ctrlMode = 'COMPASS';
            disp('COMASS恢复有解，控制权回到COMPASS')
        end
    else
        if strcmp(ctrlMode,'COMPASS')
            traci.vehicle.setLaneChangeMode(ego.vehID, 0b101010101010); % 把控制权交还给SUMO
            traci.vehicle.setSpeedMode(ego.vehID,0b011111); % bit5 bit4 ... bit0. bit0是考虑安全速度，把其勾选上
            traci.vehicle.setSpeed(ego.vehID,-1);
            ctrlMode = 'SUMO';
            disp('COMPASS无解，将控制权交回SUMO')
        end
    end
    spdDesCmds = max(theScene.getSpdDesCmds(sampleTime),1);
    routeRenewHight = 0;
    spdDesCmdDeviationStep = 1;% 原来是1，我改成10，就是提前1s后的速度给到车辆
    lastDecisionTimeGap = 0; % 上次决策的时间置零
    LC_decision = theScene.getDecision;
    if LC_decision > 0
        traci.vehicle.changeSublane(ego.vehID,3.2*(1-theScene.getEgoInitIdxDev));
    elseif LC_decision < 0
        traci.vehicle.changeSublane(ego.vehID,-3.2*(1-theScene.getEgoInitIdxDev));
    else % 当前没有换道指令，但是没在中心线行驶，那就换到车道中线去
        theDev = theScene.getEgoInitIdxDev;
        if abs(theDev) > 0.1
            traci.vehicle.changeSublane(ego.vehID,-3.2*theScene.getEgoInitIdxDev);
        end
    end
else
    disp(repmat('#',1,60));
    disp([repmat('#',1,20) '当前正在换道，跳过一次规划' repmat('#',1,20)]);
    disp(repmat('#',1,60));
end



trimmed_route = getTrimmedRoute(route,ego.pos(1:2),radius+150,ego.heading_cos_sin,scenario.Actors(1,1).Length*-3.5);
set(compassRoute_handle,'XData',trimmed_route(:,1), ...
    'YData',trimmed_route(:,2),'ZData',0*linspace(0.5,10,length(trimmed_route)))
% set(compassRoute_handle,'XData',trimmed_route(:,1), ...
%     'YData',trimmed_route(:,2),'ZData',trimmed_route(:,3))

function [route,theScene,results] = callParallel_DynMC_SimTree(s,coreNum)
disp(repmat('-',2,60))
disp([repmat('-',1,20) 'DynMC-SimTree Search' repmat('-',1,20)])
disp(repmat('-',2,60))
tic
% 初始化阶段
trimmedSet = s.getTrimmedTreeSet();
balancedSet = optimizeCutting(trimmedSet, coreNum, 'disp');
results = cell(length(balancedSet)+1,1);
allcosts = inf(length(balancedSet)+1,1);

vehState = s.vehState;
vehNum = s.vehNum;
vehicles = s.vehicles;
net = s.net;
lastRoute = s.lastRouteInfo;
lastTime = s.lastDecisionTimeGap;
s.sectionSimTime = s.maxSimTime;

% 循环仿真车道保持场景
currentTime = 0;
for simRound = 1:14 % 一共分为14个仿真阶段，推演之后20s的未来
    if simRound <= 8 % 每次仿真的时间长度先密后疏
        incSimTime = 1;
    else
        incSimTime = 2;
    end
    % 设置本阶段要仿真到的时间
    currentTime = currentTime + incSimTime;
    % 设置本阶段仿真时间
    s.sectionSimTime = currentTime; 
    s = s.simulate;
end
s.allowedPattern = 'K';
laneKeep_cost = s.cost;
results{end} = s;
allcosts(end) = laneKeep_cost;
disp([repmat('-',1,20) '仿真场景树并行搜索' repmat('-',1,20)])
parfor parSceNum = 1:length(balancedSet) % 场景并行
    % parSceNum = 1;
    maxIterCaseNum = max([3 4 5 5 5  5 5 4 3 3  2 2 1 1],13); % 只提取每阶段最优的n个场景展开 35→3
    currentTime = 0; 
    validCounter = 1; % 有效场景数量
    % scenarioSet = {s}; % 上次仿真得到的有效场景集合
    scenarioSet = {SimScenario('cost_compare',laneKeep_cost,'vehState',vehState,'vehNum',vehNum, ...
        'vehicles',vehicles,'net',net,'lastRouteInfo',lastRoute,'lastDecisionTimeGap',lastTime)};
    scenarioSet{1} = scenarioSet{1}.initSenario;
    scenarioSet{1} = scenarioSet{1}.setAllowedPattern(balancedSet{parSceNum});
    % 循环仿真阶段
    for simRound = 1:14 % 一共分为14个仿真阶段，推演之后20s的未来
        if ~isempty(scenarioSet)
            if simRound <= 8 % 每次仿真的时间长度先密后疏
                incSimTime = 1;
            else
                incSimTime = 2;
            end
            % 设置本阶段要仿真到的时间
            currentTime = currentTime + incSimTime;

            % 先来一遍循环，检查一下有效场景个数
            last_validCounter = validCounter;
            last_scenarioSet = scenarioSet;
            noneTrimmedIdxSet = zeros(last_validCounter,1);
            ntCounter = 0;
            for sceNum = 1:last_validCounter
                if ~last_scenarioSet{sceNum}.trim
                    ntCounter = ntCounter + 1; 
                    noneTrimmedIdxSet(ntCounter) = sceNum;
                end
            end
 
            % 统计一下上一阶段所有场景的代价，找代价最小的maxIterCaseNum个场景
            costs = zeros(ntCounter,1);
            for i = 1:ntCounter
                sceNum0 = noneTrimmedIdxSet(i);
                costs(i) = last_scenarioSet{sceNum0}.cost;
            end
            [~,idxs] = sort(costs);
            realValidCaseNum = min(ntCounter,maxIterCaseNum(simRound)); 

            % 先来一遍循环，检查一下有效场景个数
            validCounter = 0;
            for i = 1:realValidCaseNum
                sceNum0 = idxs(i);
                % 设置本阶段仿真时间
                last_scenarioSet{sceNum0}.sectionSimTime = currentTime; 
                % 计数原有场景和新拓展场景
                addSceNum = sum(last_scenarioSet{sceNum0}.ego_canLC);
                validCounter = validCounter + addSceNum + 1; 
            end
            
            % 根据统计好的有效场景数量分配一个cell
            scenarioSet = cell(validCounter,1);
   
            % 将所有内容装入cell中
            loadedSceNum = 0;
            for i = 1:realValidCaseNum
                sceNum0 = idxs(i);
                % 先把原场景装进来
                loadedSceNum = loadedSceNum + 1;
                scenarioSet{loadedSceNum} = last_scenarioSet{sceNum0}; 
                ego_canLC = last_scenarioSet{sceNum0}.ego_canLC;
                if ego_canLC(1)
                    loadedSceNum = loadedSceNum + 1;
                    scenarioSet{loadedSceNum} = last_scenarioSet{sceNum0}.setAction('L');
                end
                if ego_canLC(2)
                    loadedSceNum = loadedSceNum + 1;
                    scenarioSet{loadedSceNum} = last_scenarioSet{sceNum0}.setAction('R');
                end

            end

        
            for sceNum = 1:validCounter
               scenarioSet{sceNum} = scenarioSet{sceNum}.simulate;
            end
            

            disp(['场景' balancedSet{parSceNum} '第' num2str(simRound) '阶段，仿真到第' ...
               num2str(currentTime) 's，此时有' num2str(validCounter) '种情况'])

        end
    end
    if ~isempty(scenarioSet)
        costs = zeros(length(scenarioSet),1);
        for k = 1:length(costs)
            costs(k) = scenarioSet{k}.cost;
        end
        [minval,idx] = min(costs);
        results{parSceNum} = scenarioSet{idx};
        allcosts(parSceNum) = minval;
    end
end
% 后处理阶段
[~,min_idx] = min(allcosts);
theScene = results{min_idx};
route = theScene.getEgoRoute(3); % 3 表示获取3维的轨迹，带有时间信息

simTimeCost = toc;

disp(['场景并行广度优先搜索用时' num2str(simTimeCost) 's'])

end


