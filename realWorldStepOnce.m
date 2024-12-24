for i = 1:10
    % 进行一步仿真，需要先运行get_DynMC_SimTree_route进行规划才行哦
    spdDesCmdDeviationStep = spdDesCmdDeviationStep + 1;
    lastDecisionTimeGap = lastDecisionTimeGap + sampleTime;
    iterNum = iterNum + 1;
    tic;%-------------------------探针---------------开始计时
    thredSpd = theScene.getEgoDesSpd+5;
    if strcmp(ctrlMode,'COMPASS')
        if ego.speed < thredSpd
            spdDesCmd = spdDesCmds(spdDesCmdDeviationStep);
        else
            spdDesCmd = thredSpd;
        end
        traci.vehicle.setSpeed(ego.vehID,spdDesCmd);
    end
    traci.simulation.step() % 进行一步仿真
    allVehicleList = traci.vehicle.getIDList;
    
    if ~mod(iterNum,stepRange4ImpssbVehID)
        impsblVehList = getImpsblVehList(allVehicleList,ego.pos([1 2]),inreachableRadius);
    end
    
    
    
    step = step + 1; % 进行仿真步计数
    
    if ~mod(iterNum,10)
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
        setAxisRange(ego.pos([1 2]),radius);% 设置坐标轴范围，缩小到感兴趣的局部区域
        
        trimmed_route = getTrimmedRoute(route,ego.pos(1:2),radius+150,ego.heading_cos_sin,scenario.Actors(1,1).Length*-3.5);
        set(compassRoute_handle,'XData',trimmed_route(:,1), ...
                                'YData',trimmed_route(:,2), ...
                                'ZData',0*linspace(0.5,10,length(trimmed_route)));
        trajSet = theScene.getAllLogTraj(3);
        routeRenewHight = routeRenewHight - 0.05;
        for i = 1:length(trajSet)
            traj = trajSet{i};
            % trimmed_traj = getTrimmedRoute(traj,ego.pos(1:2),radius+150,ego.heading_cos_sin,scenario.Actors(1,1).Length*-3.5);
            set(vehTrajDummies{i}, ...
                'XData',traj(:,1), ...
                'YData',traj(:,2), ...
                'ZData',traj(:,3)+routeRenewHight)
        end
        for i = length(trajSet)+1:(maxVehNum+1)
            set(vehTrajDummies{i}, 'XData',1:2, 'YData',1:2, 'ZData',1:2)
        end
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
    end
    
    runtimeList(iterNum) = tSpend;

end