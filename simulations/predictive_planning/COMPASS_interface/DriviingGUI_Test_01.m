close all
clc
% clear
% load ProcessedMap_new_yizhuang.mat

%% 启动SUMO仿真
startSim = sqrt(1);
ifOutGIF = sqrt(1);
gifName = 'NewYizhaungTestRun_01.gif';
gifDelayTime = 0.1;
vehicleID = 't_0';
% jumpToTime = [];
jumpToTime = 80; % 跳到第几秒
if startSim
    sumoBinary = 'sumo-gui'; % 或者使用'sumo'进行无GUI仿真  YizhuangSim SimulateOneRout
    sumoCmd = [sumoBinary ' -c "E:\SUMO_FILES\MyYizhuangSim.sumocfg"' ' --start'];
    traci.start(sumoCmd);
    
    
    % 设置SUMO GUI的视角和跟踪车辆
    % 设置SUMO GUI的显式模式为'real world'
    viewID = 'View #0'; % 默认视角ID
    schema = 'real world';
    traci.gui.setSchema(viewID, schema);
    
    % 设置放大倍数
    zoomLevel = 10000; % 根据需要调整放大倍数
    traci.gui.setZoom(viewID, zoomLevel);
    if isempty(jumpToTime)
        traci.simulationStep(); % 进行一步仿真
    else
        while traci.simulation.getTime()<jumpToTime
            traci.simulationStep(); % 进行一步仿真
        end
    end
    vehicleIDs = traci.vehicle.getIDList(); % 获取所有车辆ID
    % 聚焦到指定车辆
    traci.gui.trackVehicle(viewID, vehicleID);
end
%% 导入地图
% load ProcessedMap_yizhuang.mat
figureID = 1; % 用来打开自驾可视化界面GUI
resultFigID = 2; % 用于展示运行时间结果
% 添加光源
hLight = light('Position',[10 30 200],'Style','infinite');
% 设置材质属性
material dull;
% 设置照明
lighting gouraud;
%% 定义绘图信息
plotReq_targetLane = PlotReq('color',[0,0.05,0.08],'faceAlpha',0.55,'height',0.03,'lineWidth',0.5,'edgeColor',[1 1 1]);
plotReq_edge = PlotReq('color','k','faceAlpha',0.6,'height',0,'lineWidth',0.5,'edgeColor',[1 1 1]);
plotReq_junction = PlotReq('color',[0 0.1 0.1],'faceAlpha',0.7,'height',-0.001);
plotReq_dirArrow = PlotReq('color','w','faceAlpha',0.9,'height',0.001,'lineWidth',0.5,'edgeColor',[1 1 1]);

%% 构建道路场景
%场景初始化
sampleTime = 0.1; %修改采样时间
scenario = drivingScenario; %初始化场景
scenario.SampleTime = sampleTime;  


%% 构建车辆信息
% 定义用于画面显示的虚拟动力学
tau_heading = 0.3;
% vehicleLength = traci.vehicle.getLength(vehicleID);
sensingFront = 50; % 假如感知范围是一个圆形，那么我关注的是自车前方sensingFront为中心的一个圆形
radius = 200; % 显示半径，配合center中心偏移，显示本车前方以center为中心radius半径的圆形区域
%% 获取自车全局路径
globalRoute = traci.vehicle.getRoute(vehicleID);


%% 在场景中添加车辆
%生成初始化的周车
maxVehNum = 15;
vehicleDummies = cell(1,maxVehNum);
initHeading_cos_sin = [1,0];
initPos = zeros(1,3);
initWayPoints = [initPos(1,[1 2]);1,1];
initSpeed = 1e-2;
initLanePosition = 0;

%根据自车信息添加车辆
ego = VehicleDummy('laneID','notSpecified','edgeID','notSpecified','lanePosition',initLanePosition,'speed',initSpeed,...
        'waypoints',initWayPoints,'heading_cos_sin',initHeading_cos_sin,...
        'pos',initPos,'vehicle',vehicle(scenario,'ClassID',1,...
        'Position',initPos,'Mesh',driving.scenario.carMesh));

% ego.vehicle.PlotColor = [0.6 0.5 1];% 紫色
ego.vehicle.PlotColor = [1 1 1];% 白色
ego = updataVehicleData(ego,entity_dict,vehicleID,sampleTime);
trajectory(ego.vehicle,ego.waypoints,ego.speed);

for idx = 1:maxVehNum
    vehicleDummies{idx} = VehicleDummy('laneID','notSpecified','lanePosition',initLanePosition,'speed',initSpeed,...
        'waypoints',initWayPoints,'heading_cos_sin',initHeading_cos_sin,...
        'pos',initPos,'vehicle',vehicle(scenario,'ClassID',1,...
        'Position',initPos,'Mesh',driving.scenario.carMesh));
    vehicleDummies{idx}.vehicle.PlotColor = [1 1 0];
    trajectory(vehicleDummies{idx}.vehicle,initWayPoints,initSpeed);
end


%% 绘制GUI初始化
maxArrowNum = 5;
COMPASS_gui = figure(figureID);
[edgeList, junctionList] = getEntityInRange(proxyMat,ego.pos([1 2]),radius,ego.edgeID);
edge_handle_dict = plot3SUMOentity(entity_dict,edgeList,figureID,plotReq_edge);
junction_handle_dict = plot3SUMOentity(entity_dict,junctionList,figureID,plotReq_junction);
ground_handle = plotGroundPlane([0.4,0.5,0.55],ego.pos([1 2]),-0.1); % 绘制地面
arrowList = queryDirArrow_withinRadius(dirArrowMap,ego.pos([1 2]),radius,maxArrowNum);
arrow_handle_dict = plot3DirectionArrow(dirArrowMap,figureID,arrowList,plotReq_dirArrow);

chasePlot(scenario.Actors(1,1),'Parent', gca,...
    'meshes','On','ViewHeight',scenario.Actors(1,1).Height*3,...
    'ViewPitch',10,'ViewLocation',[scenario.Actors(1,1).Length*-3.5,0]);
% set(gcf,'Color',0.5*ones(1,3)) % 深灰
% set(gcf,'Color',[0.2 0.23 0.23]*2) % 深蓝灰
set(gcf,'Color',[0.4 0.6 0.65]) % 深蓝灰,更蓝
mainAxes = gca;
egoRing = VehicleRing('num_points',20,'center',ego.pos([1 2]),'figureID',figureID); % 绘制自车的光环
egoLight = VehicleSignalLight('figureID',figureID); % 画出制动和转向灯 
egoLight.refreshPosition(ego.pos,ego.heading); % 把灯的姿态调整一下
egoBrkLightState = 0;
egoTurnLightState = 0; % -1,0,1 右,灭,左
egoSpeedDisp = Dashboard_Speed('figureID',figureID,'smallMap',smallMap); 
egoSpeedDisp.refreshMap(ego.pos,ego.heading,radius*3)
egoSpeedDisp.setSpeed(ego.speed);
[routeRelation,remainDist]=getNextRoutePointInfo(entity_dict,connection_dict,globalRoute,ego.laneID,ego.lanePosition);
egoSpeedDisp.setNavigation(routeRelation,remainDist);
% 绘制转向灯
if ego.changeLane > 0 || (ego.nearJunction && (strncmpi(routeRelation,'l',1) || strncmpi(routeRelation,'t',1)))
    % 该打左转向灯了，右转向灯应该关上
    if egoTurnLightState ~= 1
        egoTurnLightState = 1;
        egoLight.setTrunLight('left');
    end
elseif ego.changeLane < 0 || (ego.nearJunction && strncmpi(routeRelation,'r',1))
    % 该打右转向灯了，左转向灯应该关上
    if egoTurnLightState ~= -1
        egoTurnLightState = -1;
        egoLight.setTrunLight('right');
    end
else % 都应该关上啦
    if egoTurnLightState ~= 0
        egoTurnLightState = 0;
        egoLight.setTrunLight('off');
    end
end
% 重新编辑主界面Axes使其成为默认Axes
set(COMPASS_gui,'CurrentAxes',mainAxes)
egoTargetLaneID_stored = ego.getTargetLaneID;
target_lane_handle = plot3SUMOentity(entity_dict,{ego.getTargetLaneID},figureID,plotReq_targetLane);





%% 初始化对象跟踪器
oldVehicleList = [];
availlableObjQueue = Queue_Cell(); % 还没有赋值的实体
availlableObjQueue.elements = cellfun(@(x) x, num2cell(1:maxVehNum), 'UniformOutput', false);
availlableObjQueue.count = length(availlableObjQueue.elements);
objTracking_dict= dictionary(); % 对象跟踪器，用id来映射到vehicles的序号


%% 对象感知与跟踪

center = ego.pos(1,[1 2])+ego.heading_cos_sin*sensingFront;
allVehicleList = traci.vehicle.getIDList;
[vehicleList,vehiclePos] = ...
    findNearest_N_Vehicles(allVehicleList,vehicleID,center,maxVehNum,radius);

addVehicleList = setdiff(vehicleList,oldVehicleList); % 新加入的，需要维护出队入队，更新位置
outVehicleList = setdiff(oldVehicleList,vehicleList); % 退出的，需要更新位置到默认点并更新关系
existingVehicleList = intersect(vehicleList,oldVehicleList); % 之前就有现在还有的，需要更新位置


%%%%%% 退出的，需要更新位置到默认点并更新关系
if ~isempty(outVehicleList)
    for j = 1:length(outVehicleList)
        deleteID = outVehicleList{j};
        entityIdx = objTracking_dict(deleteID);
        availlableObjQueue = availlableObjQueue.enqueue(entityIdx);% 把可用实体还回去
        objTracking_dict(deleteID) = [];% 删除掉最早的车辆与实体的对应关系
    
        trajectory(vehicleDummies{entityIdx}.vehicle,initWayPoints,initSpeed);
    end
end

%%%%%% 新加入的，需要维护出队入队，更新位置
if ~isempty(addVehicleList)
    for j = 1:length(addVehicleList)
 
        addVehID = addVehicleList{j};
        [availlableObjQueue,entityIdx] = availlableObjQueue.dequeue();% 分配一个可用实体出去
        objTracking_dict(addVehID) = entityIdx;% 建立新加入车辆与显示实体的对应关系

        vehicleDummies{entityIdx} = ...
            updataVehicleData(vehicleDummies{entityIdx},entity_dict,addVehID,sampleTime);
        trajectory(vehicleDummies{entityIdx}.vehicle,...
            vehicleDummies{entityIdx}.waypoints,vehicleDummies{entityIdx}.speed);
    end
end


%%%%%% 之前就有现在还有的，需要更新位置
if ~isempty(existingVehicleList) 
    for j = 1:length(existingVehicleList)
        existID = existingVehicleList{j};
        entityIdx = objTracking_dict(existID);
        vehicleDummies{entityIdx} = ...
            updataVehicleData(vehicleDummies{entityIdx},entity_dict,existID,sampleTime);
        trajectory(vehicleDummies{entityIdx}.vehicle,...
            vehicleDummies{entityIdx}.waypoints,vehicleDummies{entityIdx}.speed); 
    end
end


setAxisRange(ego.pos([1 2]),radius+150);% 设置坐标轴范围，缩小到感兴趣的局部区域
pause(0.01)

if ifOutGIF
    theFrame = getframe(gcf); %获取影片帧
    [I,map]=rgb2ind(theFrame.cdata,256); %RGB图像转索引图像
    imwrite(I,map,gifName,'DelayTime',gifDelayTime,'LoopCount',Inf) %gif图像无限循环
end












% simTime = 200; % 秒
simTime = 204; % 秒
refresh_stepInterval = 3; % 每3次迭代刷新一次屏幕，基本不影响运行时间，不导致运行时间逐渐增加的问题
arrow_refreshStepInterval = 5; % 刷新箭头的步数间隔
edge_junction_refreshStepInterval = 7;% 刷新路面的步数间隔
ring_refreshStepInterval = 1; % 车辆光环刷新的步数间隔
totalStepNum = simTime/sampleTime;
runtimeList = zeros(totalStepNum,1);% 初始化仿真时间计数

timeRange4ImpssbVehID = 5; % 这些时间间隔之后会更新一次不可能到达的周车状态
maxSpeedLimit = 120/3.6;% 最高时速限制
safetyCoeff = 1.5; % 不可达距离安全系数
% 不可达半径，2车相向全力冲刺的距离再乘上安全系数
inreachableRadius = timeRange4ImpssbVehID*maxSpeedLimit*2*safetyCoeff;
stepRange4ImpssbVehID = round(timeRange4ImpssbVehID/sampleTime); % 每过这些步更新一次不可达周车表
impsblVehList = cell(0);
step = 0; % 初始化仿真时间步
gui_step = 1; % 画面更新的步数
for iterNum = 1:totalStepNum
    tic;%-------------------------探针---------------开始计时
    traci.simulation.step() % 进行一步仿真
    allVehicleList = traci.vehicle.getIDList;
    
    if ~mod(iterNum,stepRange4ImpssbVehID)
        impsblVehList = getImpsblVehList(allVehicleList,ego.pos([1 2]),inreachableRadius);
    end
    

    if ~max(ismember(allVehicleList,vehicleID)) % 如果被控本车到终点了就结束啦
        traci.close(); % 结束仿真
        runtimeList(step+1:end) = [];
        break
    end
    
    
    step = step + 1; % 进行仿真步计数
    
    if ~mod(iterNum,10) %→
    % 获取车辆相关状态
    ego = updataVehicleData(ego,entity_dict,vehicleID,sampleTime,tau_heading);
    
    

    if ~mod(iterNum,edge_junction_refreshStepInterval)
        edgeListOld = edgeList; % 保存上一步的实体表
        junctionListOld = junctionList;
        [edgeList, junctionList] = getEntityInRange(proxyMat,ego.pos([1 2]),radius,ego.edgeID); % 找到新的相关实体
        
        outEdgeList = setdiff(edgeListOld,edgeList); % 找到不再需要显示的实体
        outJunctionList = setdiff(junctionListOld,junctionList);
        edge_handle_dict = deletePlotEntity(entity_dict,outEdgeList,edge_handle_dict);% 删除这些实体
        junction_handle_dict = deletePlotEntity(entity_dict,outJunctionList,junction_handle_dict);
    
        addEdgeList = setdiff(edgeList,edgeListOld); % 找到需要新添加的实体
        addJunctionList = setdiff(junctionList,junctionListOld);
        edge_handle_dict_add = plot3SUMOentity(entity_dict,addEdgeList,figureID,plotReq_edge); % 画出这些实体
        junction_handle_dict_add = plot3SUMOentity(entity_dict,addJunctionList,figureID,plotReq_junction);
        edge_handle_dict = unionDictionary(edge_handle_dict,edge_handle_dict_add); % 更新一下现有的图形dict
        junction_handle_dict = unionDictionary(junction_handle_dict,junction_handle_dict_add);
    end
    % 箭头相关绘制
    if ~mod(iterNum,arrow_refreshStepInterval)
        arrowListOld = arrowList;
        arrowList = queryDirArrow_withinRadius(dirArrowMap,ego.pos([1 2]),radius*0.2,maxArrowNum);
        outArrowList = setdiff(arrowListOld,arrowList);
        addArrowList = setdiff(arrowList,arrowListOld);
        arrow_handle_dict = deletePlotEntity(entity_dict,outArrowList,arrow_handle_dict);
        arrow_handle_dict_add = plot3DirectionArrow(dirArrowMap,figureID,addArrowList,plotReq_dirArrow);%%%%
        arrow_handle_dict = unionDictionary(arrow_handle_dict,arrow_handle_dict_add);
    end
    % 车辆状态相关绘制
    if ~mod(iterNum,ring_refreshStepInterval) % 绘制车辆光环和刹车灯
        gui_step = gui_step + 1;
        % 绘制光环
        egoRing.showAnimation_at_step(gui_step,ego.pos([1 2]))
        % 绘制刹车灯
        egoLight.refreshPosition(ego.pos,ego.heading); % 把灯的姿态调整一下
        if ego.acc < -1
            if ~egoBrkLightState
                egoBrkLightState = 1;
                egoLight.setBrakeLight('on');
            end
        else
            if egoBrkLightState
               egoBrkLightState = 0;
               egoLight.setBrakeLight('off');
            end
        end
        
        % 绘制速度显示
        egoSpeedDisp.setSpeed(ego.speed);
        [routeRelation,remainDist]=getNextRoutePointInfo(entity_dict,connection_dict,globalRoute,ego.laneID,ego.lanePosition);
        egoSpeedDisp.setNavigation(routeRelation,remainDist);
        % 绘制转向灯
        if ego.changeLane > 0 || (ego.nearJunction && (strncmpi(routeRelation,'l',1) || strncmpi(routeRelation,'t',1)))
            % 该打左转向灯了，右转向灯应该关上
            if egoTurnLightState ~= 1
                egoTurnLightState = 1;
                egoLight.setTrunLight('left');
            elseif egoTurnLightState == 1
                egoTurnLightState = 0;
                egoLight.setTrunLight('off');
            end
        elseif ego.changeLane < 0 || (ego.nearJunction && strncmpi(routeRelation,'r',1))
            % 该打右转向灯了，左转向灯应该关上
            if egoTurnLightState ~= -1
                egoTurnLightState = -1;
                egoLight.setTrunLight('right');
            elseif egoTurnLightState == -1
                egoTurnLightState = 0;
                egoLight.setTrunLight('off');
            end
        else % 都应该关上啦
            if egoTurnLightState ~= 0
                egoTurnLightState = 0;
                egoLight.setTrunLight('off');
            end
        end

        new_egoTargetLaneID = ego.getTargetLaneID;
        if ~strcmp(egoTargetLaneID_stored,new_egoTargetLaneID) % 如果发现目标车道和之前不一样啦，要重新画
            egoTargetLaneID_stored = new_egoTargetLaneID;
            keys = target_lane_handle.keys;
            key = keys{1};
            delete(target_lane_handle(key));
            target_lane_handle = plot3SUMOentity(entity_dict,{ego.getTargetLaneID},figureID,plotReq_targetLane);
        end
    end
    if ~mod(iterNum,ring_refreshStepInterval*5) % 绘制小地图更新
        egoSpeedDisp.refreshMap(ego.pos,ego.heading,radius*3)
    end

    trajectory(ego.vehicle,ego.waypoints,ego.speed); % 更新自车位置
    


    %% 对象感知与跟踪

    
    oldVehicleList = vehicleList;
    center = ego.pos(1,[1 2])+ego.heading_cos_sin*sensingFront;
    vehicleList2Check = setdiff(allVehicleList,impsblVehList); 
    [vehicleList,vehiclePos] = ...
        findNearest_N_Vehicles(vehicleList2Check,vehicleID,center,maxVehNum,radius);
    
    addVehicleList = setdiff(vehicleList,oldVehicleList); % 新加入的，需要维护出队入队，更新位置
    outVehicleList = setdiff(oldVehicleList,vehicleList); % 退出的，需要更新位置到默认点并更新关系
    existingVehicleList = intersect(vehicleList,oldVehicleList); % 之前就有现在还有的，需要更新位置
    
    
    %%%%%% 退出的，需要更新位置到默认点并更新关系
    if ~isempty(outVehicleList)
        for j = 1:length(outVehicleList)
            deleteID = outVehicleList{j};
            entityIdx = objTracking_dict(deleteID);
            availlableObjQueue = availlableObjQueue.enqueue(entityIdx);% 把可用实体还回去
            objTracking_dict(deleteID) = [];% 删除掉最早的车辆与实体的对应关系
        
            trajectory(vehicleDummies{entityIdx}.vehicle,initWayPoints,initSpeed);
        end
    end
    
    %%%%%% 新加入的，需要维护出队入队，更新位置
    if ~isempty(addVehicleList)
        for j = 1:length(addVehicleList)
     
            addVehID = addVehicleList{j};
            [availlableObjQueue,entityIdx] = availlableObjQueue.dequeue();% 分配一个可用实体出去
            objTracking_dict(addVehID) = entityIdx;% 建立新加入车辆与显示实体的对应关系
    
            vehicleDummies{entityIdx} = ...
                updataVehicleData(vehicleDummies{entityIdx},entity_dict,addVehID,sampleTime);
            trajectory(vehicleDummies{entityIdx}.vehicle,...
                vehicleDummies{entityIdx}.waypoints,vehicleDummies{entityIdx}.speed);
        end
    end
    
    
    %%%%%% 之前就有现在还有的，需要更新位置
    if ~isempty(existingVehicleList) 
        for j = 1:length(existingVehicleList)
            existID = existingVehicleList{j};
            entityIdx = objTracking_dict(existID);
            vehicleDummies{entityIdx} = ...
                updataVehicleData(vehicleDummies{entityIdx},entity_dict,existID,sampleTime);
            trajectory(vehicleDummies{entityIdx}.vehicle,...
                vehicleDummies{entityIdx}.waypoints,vehicleDummies{entityIdx}.speed); 
        end
    end
    setAxisRange(ego.pos([1 2]),radius+150);% 设置坐标轴范围，缩小到感兴趣的局部区域
    if ~mod(iterNum,refresh_stepInterval)
        % pause(0.001)
        % drawnow limitrate
    end
    drawnow limitrate % 自动限制画面刷新率，如果超过20Hz就会抛弃一次刷新
    if ifOutGIF
        theFrame = getframe(gcf); %获取影片帧
        [I,map]=rgb2ind(theFrame.cdata,256);
        imwrite(I,map,gifName,'WriteMode','append','DelayTime',gifDelayTime) %添加到图像
    end
    end % 对应if ~mod(iterNum,2)

    tSpend=toc;disp(['用时 ' num2str(tSpend*1000) ' ms']) % -----探针----计时结束
    if tSpend > 0.1
    disp('这里有问题！！！************************************************')
    % break
    end
    runtimeList(iterNum) = tSpend;


end

hold off
% traci.close();
%{


%}

%% 仿真时间后处理
figure(resultFigID)
subplot(2,1,1)
plot(1:step,runtimeList*1000);
hold on
windowSize = 100;%指定移动平均窗口大
smoothedRuntime = movmean(runtimeList, windowSize);% 计算移动平均值
plot(1:step,smoothedRuntime*1000,'LineWidth',1,'Color','r');
plot([1 step],1000*sampleTime*ones(1,2),'LineWidth',1,'Color','g');
hold off
legend('real runtime','smoothed','realtime limit','Location','northwest')
xlabel('step')
ylabel('runtime [ms]')
subplot(2,1,2)
histogram(runtimeList*1000,50);
xlabel('runtime [ms]')
ylabel('step number')

