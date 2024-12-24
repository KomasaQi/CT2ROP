function vehicle = updataVehicleData(vehicle,entity_dict,vehicleID,sampleTime,headingDynTau)
    if ~strcmp(vehicle.vehID,vehicleID) % 当这个实体被赋予给了一个新的vehicleID的时候，就需要更新车辆路线
        new_vehicle_flag = true;
        vehicle.vehID = vehicleID;
        vehicle.route = traci.vehicle.getRoute(vehicleID);
    else
        new_vehicle_flag = false;
    end
    vehicle.routeIdx = traci.vehicle.getRouteIndex(vehicleID)+1; % 更新车辆的routeIndex

    % vehicleLength = traci.vehicle.getLength(vehicleID);
    
    vehicleLength = 5;% 这里默认为5了，需要再改
    
    % tic
    oldLaneNo = vehicle.getLaneNo();
    laneID = traci.vehicle.getLaneID(vehicleID);
    vehicle.laneID = laneID;
    newLaneNo = vehicle.getLaneNo();

    % t1 = toc;
    
    % tic
    oldEdgeID = vehicle.edgeID;
    newEdgeID = entity_dict{laneID}.getEdgeID();
    vehicle.edgeID = newEdgeID;
    % t2 = toc;

    % tic
    veh_pos = traci.vehicle.getPosition(vehicleID);
    % t3 = toc;
    oldDev = vehicle.dev;
    [~, newDev, ~, ~] = projPoint2Polyline_mex(entity_dict{laneID}.shape, veh_pos);
    vehicle.dev = newDev;

    if new_vehicle_flag %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%需要验证新加入的2个状态是否准确,验证了，基本没问题
        vehicle.changeLane = 0;
    else
        if ~strcmp(oldEdgeID,newEdgeID) % 如果到了一条新的道
            vehicle.changeLane = 0;
        else
            if newLaneNo == oldLaneNo % 至少车道还没变
                devDiff = newDev - oldDev;
                if devDiff > 0 && newDev > 0.1 % 保证至少有一定的偏离
                    vehicle.changeLane = 1;
                elseif devDiff < 0 && newDev < -0.1
                    vehicle.changeLane = -1;
                else
                    vehicle.changeLane = 0;
                end
            elseif newLaneNo < oldLaneNo % 向右移动啦
                vehicle.changeLane = 0; % -1
            else % newLaneNo > oldLaneNo 向左移动啦
                vehicle.changeLane = 0; % 1 
            end
        end
    end 




    vehicle.targetLaneIdx = max(min(newLaneNo + vehicle.changeLane,entity_dict{newEdgeID}.laneNum-1),0);
    % [tm,idx] = max([t1 t2 t3]);
    % if tm > 0.1
    %     disp(['这里有问题！！！************************************************,t' num2str(idx) '用时' num2str(tm*1000) 'ms, vehicleID: ' vehicleID])
    % end

    dPos = veh_pos - vehicle.pos([1 2]);
    dDist = norm(dPos);
    headingAvail_Flag = false;
    dist = traci.vehicle.getLanePosition(vehicleID);
    vehicle.lanePosition = dist;
    if dDist>1e-1 && dDist < 100 
        vehicle.heading = atan2(dPos(2),dPos(1));
        headingAvail_Flag = true;
    end

    if ~headingAvail_Flag % 如果直接求heading不好用，再用笨办法求heading_cos_sin
        
        [~,heading_cos_sin] = getVehicleHeading(entity_dict,laneID,dist);
        if ~isempty(heading_cos_sin)
            if nargin == 4
                vehicle.heading_cos_sin = heading_cos_sin;
                vehicle.heading = atan2(heading_cos_sin(2),heading_cos_sin(1));
            elseif nargin == 5
                 vehicle.heading_cos_sin = ...
                     vehicle.heading_cos_sin + 1/headingDynTau*(heading_cos_sin - vehicle.heading_cos_sin)*sampleTime;
                 vehicle.heading = atan2(vehicle.heading_cos_sin(2),vehicle.heading_cos_sin(1));
            else
                error('输入参数数量不对，请参阅本函数的具体定义，喵喵~')
            end
        end

    else
        heading_cos_sin = dPos/dDist;
        if nargin < 5
            vehicle.heading_cos_sin = heading_cos_sin;
        elseif nargin == 5
             vehicle.heading_cos_sin = ...
                 vehicle.heading_cos_sin + 1/headingDynTau*(heading_cos_sin - vehicle.heading_cos_sin)*sampleTime;
        else
            error('输入参数数量不对，请参阅本函数的具体定义，喵喵~')
        end
        vehicle.heading = atan2(vehicle.heading_cos_sin(2),vehicle.heading_cos_sin(1));
    end
    new_speed = abs(traci.vehicle.getSpeed(vehicleID))+1e-2;
    vehicle.acc = traci.vehicle.getAcceleration(vehicleID);
    vehicle.speed = new_speed;
    vehicle.waypoints = [veh_pos+vehicle.heading_cos_sin*-vehicleLength*0.5;veh_pos];
    vehicle.pos=[veh_pos,0];

    if abs(vehicle.acc) < 0.6 && vehicle.speed > 10 % 一个阈值
        vehicle.isCruising = true;
    else
        vehicle.isCruising = false;
    end
    if vehicle.isCruising
        vehicle.v_des = vehicle.speed + vehicle.acc;
    else
        % 如果在交叉口内的边，其期望速度设置为下一个路径上的正常边的期望速度，否则就是当前边的期望速度
        if strncmp(':',laneID,1) 
            vehicle.v_des = entity_dict{[vehicle.route{vehicle.routeIdx} '_0']}.speed;
        else
            vehicle.v_des = entity_dict{laneID}.speed;
        end
        vehicle.isCruising = false;
 
    end

    if (entity_dict{laneID}.length- dist) < 50 % 在路口提前50m打灯
        vehicle.nearJunction = true;
    else
        vehicle.nearJunction = false;
    end





end



%% TODO:车辆的意图预测