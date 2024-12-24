simFigID = 3;
ifOutGIF = sqrt(1);
gifName = 'SurrVehSim_002.gif';
gifDelayTime = 0.1;
%{
    2.其次根据路网范围确定是关注哪些车辆：
        (1)在目标跟踪器中，这是通过距离筛选的
        (2)在上述路网中，这是根据相关性筛选的
    
    3.针对每一个筛选出来的车辆，根据边的连接关系，设置其接下来的意图树（还有r值作为置信度）。
        对象跟踪器中有每辆车的意图更新结果，根据不同意图的概率进行场景采样。
        采样scene_num个场景，每个场景中包含相同的路网信息，以及意图固定的所有actor的edge路径
        具体路径就是下一个决策点（junction）到来时（前30m,参考法规）的可行驶车道（约束）。
    
    4.根据意图采样结果对每个场景中的不同车辆设置行驶约束：
            每辆车包含距离下一个决策点的时距状态，换道意图生成（换道概率）与其强相关
           ●如果下一个意图是直行/其他，就需要在其进入下一个junction的位置所有不能直行/其他的车道放上虚拟车

        换道概率→采样换道行为→假如换道，根据换道概率大小采样换道时间（映射），换道概率大时采样出换道就是快速变道，否则是从容变道
    5.关于如何把车辆放入新的路网当中，如果还没有在路口，就直接使用当前laneID,edgeID,在路口内的路径

%}

allDict = struct('entity_dict',new_entity_dict,'connection_from_dict',connection_dict,'connection_to_dict',to_connection_dict, ...
    'lane_from_connection_dict',lane_from_connection_dict,'lane_to_connection_dict',lane_to_connection_dict);

resolution = 100; % 100m的精度
simRoadNetwork_dict = genSimRoadNetworkDict(allDict,vehicleID);

edgeID_dist = genEdgeID_dist(ego.edgeID,ego.lanePosition,resolution);

s = SimScenario('net',simRoadNetwork_dict{edgeID_dist});
vehKeys = objTracking_dict.keys;

egoState = convertVehState(new_entity_dict,lane_from_connection_dict, ...
           lane_to_connection_dict,s,ego);

s = s.addVehicle(Vehicle4COMPASS('route',ego.route,...
    'routeIdx',ego.routeIdx,'routeNum',length(ego.route),'edgeID',ego.edgeID,'laneID',ego.laneID),egoState);

for i = 1:length(vehKeys)
    vehKey = vehKeys{i};
    % 下面进行判断车辆是否在仿真路网内部
    vehNo = objTracking_dict(vehKey);
    if s.net.isInNet(vehicleDummies{vehNo}.edgeID)
        vehState = convertVehState(new_entity_dict,lane_from_connection_dict, ...
                   lane_to_connection_dict,s,vehicleDummies{vehNo});
 
        s = s.addVehicle(Vehicle4COMPASS('route',vehicleDummies{vehNo}.route,...
            'routeIdx',vehicleDummies{vehNo}.routeIdx,...
            'edgeID',vehicleDummies{vehNo}.edgeID,...
            'laneID',vehicleDummies{vehNo}.laneID, ...
            'routeNum',length(vehicleDummies{vehNo}.route)),vehState);
    end

end

s = s.initSenario();



figure(simFigID)
title('周围交通实时推演情况')
xlabel('地图相对位置x坐标[m]')
ylabel('地图相对位置y坐标[m]')
grid on
plotReq_edge0 = PlotReq('color','r','faceAlpha',0.2,'height',0,'lineWidth',0.5,'edgeColor',[1 0.6 0.6]);
hold on
% 绘制道路
plotSUMOentity(new_entity_dict,{new_entity_dict{ego.edgeID}.from},simFigID);
plotSUMOentity(new_entity_dict,s.net.edgeList,simFigID);
hold on
for i = 1:length(s.net.edgeList)
    if s.net.frindgeEdgeArray(i)
        theEdgeID = s.net.edgeList{i};
        plot3SUMOentity(new_entity_dict,{theEdgeID},simFigID,plotReq_edge0);
    end
end
% 绘制车辆
hold on
veh_text_handles = cell(s.vehNum,1);
veh_dot_handles = cell(s.vehNum,1);
veh_driveLine_handles = cell(s.vehNum,1);
for i = 1:s.vehNum
    if s.vehState(i,s.var.opsState) == 2
        veh_text_handles{i} = text(s.vehState(i,s.var.x),s.vehState(i,s.var.y),['No.' num2str(i) ', opsState=2']);
    else
        veh_text_handles{i} = text(s.vehState(i,s.var.x),s.vehState(i,s.var.y),['No.' num2str(i)]);
    end
    veh_dot_handles{i} = scatter(s.vehState(i,s.var.x),s.vehState(i,s.var.y),150,"filled");
    driveLine = s.vehDriveLine{i};
    veh_driveLine_handles{i} = plot(driveLine(:,1),driveLine(:,2),'g','LineWidth',1.5);
end

if ifOutGIF
    theFrame = getframe(gcf); %获取影片帧
    [I,map]=rgb2ind(theFrame.cdata,256); %RGB图像转索引图像
    imwrite(I,map,gifName,'DelayTime',gifDelayTime,'LoopCount',Inf) %gif图像无限循环
end


















































