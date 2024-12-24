%{
    转换车辆状态
%}
function vehState = convertVehState(new_entity_dict,lane_from_connection_dict, ...
                   lane_to_connection_dict,simScenario,vehicleDummy)
    flag = simScenario.net.isInNet(vehicleDummy.edgeID);
    pos = vehicleDummy.pos([1 2]);
    vehState = zeros(1,simScenario.stateNum);
    dead_flag = false;
    switch flag
        case {1,2} % 1：正常边，2：交叉口内的正常边
            laneNo = vehicleDummy.getLaneNo;
            line = new_entity_dict{vehicleDummy.laneID}.shape;
            [~, dev, laneDist] = projPoint2Polyline(line, pos);
            laneIdxDev = laneNo + dev/new_entity_dict{vehicleDummy.laneID}.width;
            currentLaneID = vehicleDummy.laneID;
            edgeNo = simScenario.net.edge_No_dict(vehicleDummy.edgeID);
            maxLaneIdxDev = new_entity_dict{vehicleDummy.edgeID}.laneNum - 0.5;
            
        case 3 % 3：交叉口内的二级边
            % 1.找to lane的所有from lane
            % 2.找to lane的from lane
            % 3.从new实体字典的edge的lane中找
            theLaneID = vehicleDummy.laneID;
            toLaneList = getToLaneList(lane_from_connection_dict,theLaneID);
            toLaneID = toLaneList{1};
            fromLaneList = getFromLaneList(lane_to_connection_dict,toLaneID);
            toCheckLaneList = setdiff(fromLaneList,{theLaneID}); % 候选的lane列表
            
            totErr = inf;
            projEdgeID = [];
            for i = 1:length(toCheckLaneList)
                checkLaneID = toCheckLaneList{i};
                line = new_entity_dict{checkLaneID}.shape;
                [~, dev_prep, laneDist_prep, refHead] = projPoint2Polyline(line, pos);
                heading_prep = atan2(refHead(2),refHead(1));
                headErr = abs(heading_prep-vehicleDummy.heading);
                devErr = abs(dev_prep);
                totErr_prep = headErr*2 + devErr;% 这里的权重可以调节
                new_flag = simScenario.net.isInNet(new_entity_dict{checkLaneID}.getEdgeID());
                if new_flag == 3 || ~new_flag 
                    totErr_prep = inf;
                end
                if totErr_prep < totErr
                    dev = dev_prep;
                    laneDist = laneDist_prep;
                    projLaneID = checkLaneID;
                    projEdgeID = new_entity_dict{projLaneID}.getEdgeID();
                end
            end
            % new_flag = simScenario.net.isInNet(projEdgeID);
            if isempty(projEdgeID) %|| new_flag == 3 || ~new_flag 
                % 如果没找到这个投影边，或者找到的还是二级边，或者根本不在网络，都认为寻找失败，将此车置为失效
                dead_flag = true;
                laneIdxDev = 0;
                edgeNo = 0;
                currentLaneID = 'Invalid_by_Komasa';
                maxLaneIdxDev = 0.5;
            else
    
                lastUnderscoreIdx = find(projLaneID=='_',1,'last');
                laneNo = str2double(projLaneID(lastUnderscoreIdx+1:end));
                laneIdxDev = laneNo + dev/new_entity_dict{projLaneID}.width;
                
                edgeNo = simScenario.net.edge_No_dict(projEdgeID);
                
                currentLaneID = projLaneID;
    
                maxLaneIdxDev = new_entity_dict{projEdgeID}.laneNum - 0.5;
            end

        otherwise
            error([vehicleDummy.vehID '的edgeID不属于输入的simScenario的simRoadNetwork!主人检查一下嘛，好不好嘛'])
    end
    
    if ~dead_flag
        isInFinalEdge = simScenario.net.frindgeEdgeArray(edgeNo);
        isInEndEdge = (vehicleDummy.routeIdx == length(vehicleDummy.route)); % 看下是不是到估计的路线的最终边啦
        % if ~isInEndEdge
        %     % 如果不是，再看看下一条边是不是在本场景的路网外部
        %     isNextEdgeOutRange = ~simScenario.net.isInNet(vehicleDummy.route{vehicleDummy.routeIdx+1});
        % end
        
        if strncmp(currentLaneID,':',1)
            opsState = 3;
        elseif isInEndEdge || isInFinalEdge %(isInFinalEdge && isNextEdgeOutRange) 这里修改了一下，因为存在下一个边在路网，但是中间缺少Junction的情况，就找不到键啦
            opsState = 2;
        else 
            opsState = 1;
        end
    else
        opsState = 0;
    end


    vehState(simScenario.var_laneIdxDev) = laneIdxDev;
    vehState(simScenario.var_laneDist) = laneDist;
    vehState(simScenario.var_x) = pos(1);
    vehState(simScenario.var_y) = pos(2);
    vehState(simScenario.var_spd) = vehicleDummy.speed;
    vehState(simScenario.var_acc) = vehicleDummy.acc;
    vehState(simScenario.var_heading) = vehicleDummy.heading;
    vehState(simScenario.var_edgeNo) = edgeNo;
    vehState(simScenario.var_opsState) = opsState;
    vehState(simScenario.var_maxLaneIdxDev) = maxLaneIdxDev; 
    vehState(simScenario.var_routeIdx) = vehicleDummy.routeIdx; 
    vehState(simScenario.var_changingLane) = vehicleDummy.changeLane;  
    vehState(simScenario.var_targetLaneIdx) = vehicleDummy.targetLaneIdx; 
    
end