% 进行仿真的步进迭代
function obj = step(obj) 
    % 进行一步仿真的步骤：
    %{
        1：找出所有车的  左前  前  右前 车的No，-1为还未检查，无车为0，虚拟车为666，无法换道则为999
                        左后  后  右后       （全部运行一遍之后不应该有-1）
    %}
    theTimeStep = obj.timeStep;

    vehicleState = obj.vehState; % 先继承上一个的vehState
    spd0s = vehicleState(:,obj.var_spd);
    acc0s = vehicleState(:,obj.var_acc);
    remDist0s = vehicleState(:,obj.var_remDist);
    laneDist0s = vehicleState(:,obj.var_laneDist);
    opsState0s = vehicleState(:,obj.var_opsState);
    laneIdxDev0s = vehicleState(:,obj.var_laneIdxDev);
    maxLaneIdxDev0s = vehicleState(:,obj.var_maxLaneIdxDev);
    % laneIdxDev0s(laneIdxDev0s<-0.5) = 0; % 加了一条处理，就是很偏，那就别偏了，正回来到正中间
    targetLaneIdx0s = max(vehicleState(:,obj.var_targetLaneIdx),0);% 这里做了修改，因为不清楚那里偶尔会导致目标车道为-1
    devSpd0s = vehicleState(:,obj.var_devSpd);


    
    % 更新状态
    % 仿真增加一步

    spds = max(spd0s + acc0s*theTimeStep,1e-3); % spd
    vehicleState(:,obj.var_spd) = spds;
    opsStates = opsState0s;
    % remDist_
    % laneDist_
    travelDists = spds*theTimeStep + acc0s*theTimeStep*theTimeStep/2; % 认为是匀加速
    remDists = remDist0s - travelDists;
    laneDists = laneDist0s + travelDists;
    toNextEdgeVehList = find(remDists < 0);
    for i = 1:length(toNextEdgeVehList) % 需要调节的
        vNo = toNextEdgeVehList(i);
        if opsState0s(vNo) % 对没有结束的车辆
            % 先看一下是否是快要结束的，是否有下一个edge
            switch opsState0s(vNo) % 如果没有下个edge了，就要结束啦
                case 2
                    opsStates(vNo) = 0;
                    laneDists(vNo) = laneDist0s(vNo) + remDist0s(vNo);
                    remDists(vNo) = 0;
                    % x_
                    vehicleState(vNo,obj.var_x) = 0; % 状态归零
                    % y_
                    vehicleState(vNo,obj.var_y) = 0;
                    % heading_
                    vehicleState(vNo,obj.var_heading) = 0;
                    % disp(['车辆 No.' num2str(vNo) ' 结束行驶！'])

                case {1,3}
                    notContinuableFlag = false;
                    laneDists(vNo) = -remDists(vNo);
                    if opsState0s(vNo) == 3 % 在交叉口内部
                        % routeIdx_
                        routeIdx = vehicleState(vNo,obj.var_routeIdx) + 1;
                        % edgeNo_
                        edgeID = obj.vehicles{vNo}.route{routeIdx};
                    else % 在正常边行驶，且有下一个edge
                        % routeIdx_
                        routeIdx = vehicleState(vNo,obj.var_routeIdx); % routeIdx不变
                        currentEdgeID = obj.edgeIDs{vNo};
                        nextRouteEdgeID = obj.vehicles{vNo}.route{routeIdx+1};
                        connections = obj.net.c_from_dict{currentEdgeID}.connections;
                        for j = 1:obj.net.c_from_dict{currentEdgeID}.connection_num
                            if strcmp(connections{j}.to,nextRouteEdgeID)
                                viaLaneID = connections{j}.via;
                                nextEdgeID = regexprep(viaLaneID, '_\d+$', '');
                                break
                            end
                        end
                        edgeID = nextEdgeID;
                    end
                    % obj.edgeIDs{vNo} = edgeID;
                    edgeNo = obj.net.edge_No_dict(edgeID);
                    remDists(vNo) = obj.net.e_dict{[edgeID '_0']}.length - laneDists(vNo);
                    % maxLaneIdxDev_
                    maxLaneIdxDev = obj.net.e_dict{edgeID}.laneNum - 0.5;

                    vehicleState(vNo,obj.var_routeIdx) = routeIdx;
                    vehicleState(vNo,obj.var_edgeNo) = edgeNo;
                    vehicleState(vNo,obj.var_maxLaneIdxDev) = maxLaneIdxDev;
                    % dir_
                    routeNum = obj.vehicles{vNo}.routeNum;
                    if routeIdx < routeNum % 如果没有后续啦，认为车将在本edge末停车,否则↓%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%可以认真修改的地方
                        % 如果是在交叉口内，说明一定还有下一个edge
                        % 在正常边内，并不是要结束了，还有下一个边
                        nextEdgeID = obj.vehicles{vNo}.route{routeIdx+1}; % 获取route中的下一个edge
                        dir_str = obj.net.getDirE2E(edgeID,nextEdgeID);
                        switch dir_str
                            case 's'
                                dir = 3;
                            case 'r'
                                dir = 2;
                            case 'l'
                                dir = 1;
                            case 't'
                                dir = 0;
                            otherwise
                                dir = -1; % 先用-1 初始化
                        end
                    else
                        dir = 3;
                    end
                    vehicleState(vNo,obj.var_dir) = dir;

                    % opsState
                    isInFinalEdge = obj.net.frindgeEdgeArray(edgeNo);
                    isInEndEdge = (routeIdx >= routeNum); % 看下是不是到估计的路线的最终边啦
                    if ~isInEndEdge
                        % 如果不是，再看看下一条边是不是在本场景的路网外部
                        isNextEdgeOutRange = ~obj.net.isInNet_fast(obj.vehicles{vNo}.route{routeIdx+1});
                    end
                    if strncmp(edgeID,':',1)
                        opsState = 3;
                    elseif isInEndEdge || (isInFinalEdge && isNextEdgeOutRange) 
                        % 2 的含义就是，要么在这辆车自己的结束，要么因为路网裁剪的原因被迫结束
                        opsState = 2;
                    else 
                        opsState = 1;
                    end
                    opsStates(vNo) = opsState;
                    



                    % targetLaneIdx_
                    targetLaneID0 = [obj.edgeIDs{vNo} '_' char('0'+targetLaneIdx0s(vNo))];

                    if opsState0s(vNo) == 3
                        % 那么当前的routeIdx对应的是已经行驶过的边，且一定存在下一个正常边
                        % 且对于交叉口内的车道来说，可以到达的下一个车道是唯一的
                        targetLaneID = obj.net.lc_from_dict{targetLaneID0}{1}.to;
                    else % 也没有马上结束，那么就是还有下一个边，以及下一个边的中间边
                        % 需要找到从本车道到下一个edge可能的lane连接，并选取其中最近的lane作为目标lane
                        % #要修改
                        line1 = obj.net.e_dict{targetLaneID0}.shape;
                        dist_min = inf;
                        idx = 0;
                        for j = 1:length(obj.net.lc_from_dict{targetLaneID0})
                            toLaneID_temp = obj.net.lc_from_dict{targetLaneID0}{j}.to;
                            toEdgeID_temp = regexprep(toLaneID_temp, '_\d+$', '');
                            nextEdgeID = obj.getNextRouteEdgeID(vNo);
                            if strcmp(toEdgeID_temp,nextEdgeID) % 需要保证这个连接的下一个edge是route中的下一个edge
                                lanePos = obj.net.e_dict{toLaneID_temp}.shape(1,:);
                                dist_xy = norm(lanePos - line1(end,:));
                                if dist_xy < dist_min
                                    idx = j;
                                end
                            end
                        end
                        if ~idx
                            disp(['vNo:' num2str(vNo) '无法找到与route中下一个目标edge内lane的连接,currentlaneID=' targetLaneID0]);
                            notContinuableFlag = true;
                        else
                            targetLaneID = obj.net.lc_from_dict{targetLaneID0}{idx}.via;
                        end
                    end
                    if ~notContinuableFlag
                        lastUnderscoreIdx = find(targetLaneID=='_',1,'last');
                        targetLaneIdx = str2double(targetLaneID(lastUnderscoreIdx+1:end));
                        vehicleState(vNo,obj.var_targetLaneIdx) = targetLaneIdx;
    
                        inc_targetLaneIdx = targetLaneIdx - targetLaneIdx0s(vNo);
                        theLaneIdxDev0 = laneIdxDev0s(vNo);
                        if theLaneIdxDev0 < -0.5
                            theLaneIdxDev0 = 0;
                        end
                        laneIdxDev0s(vNo) = theLaneIdxDev0 + inc_targetLaneIdx; % 经过转换到新的lane之后的laneIdxDev
                        devSpd0s(vNo) = devSpd0s(vNo) + inc_targetLaneIdx*2*obj.dev_zeta*obj.dev_omega;
                        if vNo == 1
                            obj.egoTargetLaneIdx = obj.egoTargetLaneIdx + inc_targetLaneIdx;
                        end
                    else
                        opsStates(vNo) = 0;
                        laneDists(vNo) = laneDist0s(vNo) + remDist0s(vNo);
                        remDists(vNo) = 0;
                        % x_
                        vehicleState(vNo,obj.var_x) = 0; % 状态归零
                        % y_
                        vehicleState(vNo,obj.var_y) = 0;
                        % heading_
                        vehicleState(vNo,obj.var_heading) = 0;
                        % disp(['车辆 No.' num2str(vNo) ' 结束行驶！'])
                    end

            end
        end
    end

    vehicleState(:,obj.var_remDist) = remDists;
    vehicleState(:,obj.var_laneDist) = laneDists;
    vehicleState(:,obj.var_opsState) = opsStates;
    targetLaneIdxs = vehicleState(:,obj.var_targetLaneIdx);
    
    
    % laneIdxDev_
    laneIdxDev0s(laneIdxDev0s < -0.5) = 0;
    % overflowIdx = laneIdxDev0s > maxLaneIdxDev0s;
    % laneIdxDev0s(overflowIdx) = maxLaneIdxDev0s(overflowIdx)-0.5;
    % devSpd0s(overflowIdx) = (maxLaneIdxDev0s(overflowIdx)-0.5)*2*obj.dev_zeta*obj.dev_omega;
    devSpds = devSpd0s + theTimeStep*(targetLaneIdxs - laneIdxDev0s)*(obj.dev_omega^2);
    laneIdxDevs = laneIdxDev0s + theTimeStep*(devSpd0s - 2*obj.dev_zeta*obj.dev_omega*laneIdxDev0s);
    % 对其进行进一步合规性检查与饱和
    % laneIdxDevs = max(min(laneIdxDevs,obj.maxLaneIdxDev_-0.5),0);
    vehicleState(:,obj.var_devSpd) = devSpds;
    vehicleState(:,obj.var_laneIdxDev) = laneIdxDevs;
    
    % x0s = obj.x_;
    % y0s = obj.y_;
    % heading0s = obj.heading_;

    for vNo = 1:obj.vehNum
        if opsStates(vNo) % 只要车辆没结束
            % x_
            % y_
            % heading_
            edgeNo = vehicleState(vNo,obj.var_edgeNo);
            edgeID = obj.net.edgeList{edgeNo};
            line = obj.net.e_dict{[edgeID '_0']}.shape;
            [x, y] = calcVehPos_onLine_mex(line, laneDists(vNo), laneIdxDevs(vNo)*3.2);
            
            vehicleState(vNo,obj.var_x) = x;
            vehicleState(vNo,obj.var_y) = y;
        end

    end
    

    obj.vehState = vehicleState;

    obj.laneIDs = obj.getAllLaneIDs(); % 一定要先更新这个表格，后面才能用的
    obj.edgeIDs = obj.getAllEdgeIDs();
    obj.junctionIDs = obj.getAllJunctionIDs();

    obj.vehState(:,obj.var_canReachNextEdge) = obj.getAllCanReachNextEdge(); 
    obj.vehDriveLine = obj.getAllVehicleLines(); % 用了canReachNextEdge,所以一定在其后更新
    obj.nextEdgeIDs = obj.getAllNextEdgeID(); % 获取下一个edge（如果有的话，对应状态1,3）
    [obj.vehState(:,obj.var_spdLim),obj.vehState(:,obj.var_nextSpdLim)] = obj.getAllSpdLim();



    [obj.surroundVeh,obj.surrVehDist,obj.surrVehSpd] = getAllSurroundVeh(obj);
    [obj.vehState(:,obj.var_frontSpace),obj.vehState(:,obj.var_backSpace),obj.vehState(:,obj.var_frontVehSpd)] = obj.getAllSpace();
    obj.vehState(:,obj.var_desSpd) = obj.getAllSpdDes();
    

    %%%% 假设换道决策是没问题的，继续往后写利用仿真结果进行验证
    [changingLanes,targetLaneIdxs,accs] = obj.globalMOBIL(); % 获取所有车的换道决策
    obj.vehState(2:end,obj.var_changingLane) = changingLanes(2:end);
    obj.vehState(:,obj.var_targetLaneIdx) = targetLaneIdxs;
    obj.vehState(:,obj.var_acc) = accs;
    

    obj.currentStep = obj.currentStep + 1; 
    obj = obj.saveState(); % 所有阶段所有车辆状态的记录
    obj.actionTimeCounter = max(obj.actionTimeCounter - obj.timeStep,0);

end