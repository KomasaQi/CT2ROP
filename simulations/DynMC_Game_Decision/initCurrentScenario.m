new_edgeID_dist = genEdgeID_dist(ego.edgeID,ego.lanePosition,resolution);
if isKey(simRoadNetwork_dict,new_edgeID_dist)
    edgeID_dist = new_edgeID_dist;
else
    disp('当前位置没有记录新的推演地图，别担心哒，仍然使用上一个推演地图哦')
end
s = SimScenario('net',simRoadNetwork_dict{edgeID_dist});
vehKeys = objTracking_dict.keys;

egoState = convertVehState(new_entity_dict,lane_from_connection_dict, ...
           lane_to_connection_dict,s,ego);

s = s.addVehicle(Vehicle4COMPASS('L',13,'delta',4,'route',ego.route,...
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
            'routeNum',length(vehicleDummies{vehNo}.route), ...
            'v_des',vehicleDummies{vehNo}.v_des),vehState);
    end

end
if ~isempty(theScene)
    s.lastRouteInfo = route; % 保存一下上次的决策记录
    s.lastDecisionTimeGap = lastDecisionTimeGap; % 上次决策距离本次决策的时间差
end
s = s.initSenario();