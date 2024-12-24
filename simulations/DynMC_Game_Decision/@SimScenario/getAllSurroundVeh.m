% #############################################################################################################
%                                              查找周车
% #############################################################################################################
% 查找每一辆车的周车并更新列表
function [surroundVehMat,surrVehDistMat,surrVehSpdMat] = getAllSurroundVeh(obj)

    vehNumber = obj.vehNum;
    dftDist = obj.defaultDist; % 默认150 米的距离
    states = obj.vehState;
    surroundVehMat = -ones(2*vehNumber,3);
    surrVehDistMat = dftDist*ones(2*vehNumber,3);
    surrVehSpdMat = -ones(2*vehNumber,3);
    % 预先切片来加速
    laneIdxDevs = states(:,obj.var_laneIdxDev);
    laneDists = states(:,obj.var_laneDist);
    xs = states(:,obj.var_x);
    ys = states(:,obj.var_y);
    spds = states(:,obj.var_spd);
    edgeNos = states(:,obj.var_edgeNo);
    dirs = states(:,obj.var_dir);
    remDists = states(:,obj.var_remDist);
    opsStates = states(:,obj.var_opsState);
    spdLims = states(:,obj.var_spdLim);
    maxLaneIdxDevs = states(:,obj.var_maxLaneIdxDev);
    canReachNextEdges = states(:,obj.var_canReachNextEdge);
    politenss = obj.politeness;
    vehicleDriveLine = obj.vehDriveLine;
    junctionIDs = obj.junctionIDs;
    edgeIDs = obj.edgeIDs;
    % 获取车辆的laneNo
    laneNos = max(min([round(laneIdxDevs),round(maxLaneIdxDevs-0.5)],[],2),0);
    % 是否靠近交叉口的标准：要么同时距离交叉口很近，要么身在交叉口当中
    nearJunctions = (remDists < 150) | (opsStates == 3); 

    % 靠近交叉路口而无法换道，也是将虚拟车设置在相应位置
    nearJunctionCanNotLCs = (remDists < 10) | (opsStates == 3); 

    latWidth = obj.latAlertWidth;
    %{
        1：找出所有车的  左前  前  右前 车的No，-1为还未检查，无车为0，虚拟车为666，无法换道则为999
                        左后  后  右后       （全部运行一遍之后不应该有-1）
    %}
    for vNo = 1:vehNumber % 对每辆车寻找周车
        opsState = opsStates(vNo);
        if opsState % 如果没有终止
            bidx = 2*(vNo-1)+1;
            currentEdgeID = edgeIDs{vNo};
            currentJunctionID = junctionIDs{vNo};
            remDist = remDists(vNo);
            nearJunction = nearJunctions(vNo);
            nearJunctionCanNotLC = nearJunctionCanNotLCs(vNo);
            laneIdxDev = laneIdxDevs(vNo);
            laneNo = laneNos(vNo);

            sameEdgeLogical = edgeNos == edgeNos(vNo);
            inLaneIdxDevs = laneIdxDevs >= (laneIdxDev - latWidth) & laneIdxDevs <= (laneIdxDev + latWidth); 
            sameEdge_inLaneIdxDevs = sameEdgeLogical & inLaneIdxDevs;
            notStoped = opsStates~=0;
            toFindList = sameEdge_inLaneIdxDevs & notStoped;
            sameLaneList = find(toFindList); % 这里是本条车道的所有车未停止的车
           
            % #################################寻找前车######################################################
            % (1)寻找前车,兼顾着在前面寻找本lane内的后车

            % 唯独前车不会被其他车的流程检查出来过，一定会走一遍这个流程
            front_veh_No = 0;
            front_veh_dist = dftDist; % 默认距离
            front_veh_spd = 0;
            sameJunctionList = [];
            % 不管哪种情况，都是先检查本条edge内正常的车
            for i = 1:length(sameLaneList)
                otherVNo = sameLaneList(i);
                if otherVNo ~= vNo % 如果不是被探查的这辆车
                    fv_dist = laneDists(otherVNo) - laneDists(vNo);
                    if fv_dist > 0 % 说明这辆车在前面
                        if fv_dist <= front_veh_dist
                            front_veh_dist = fv_dist;
                            front_veh_No = otherVNo;
                            front_veh_spd = spds(otherVNo);
                        end
                    end
                end
            end

            if front_veh_No % 如果是同一个lane的前车，那么对其来说我就是后车没得跑
                surroundVehMat(front_veh_No*2,2) = vNo;
                surrVehDistMat(front_veh_No*2,2) = front_veh_dist;
                surrVehSpdMat(front_veh_No*2,2) = spds(vNo);
            end

            switch opsState % 对应1 2 3
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                case 1 % 在正常边内，需要看是否距离交叉口足够近,比如<150m
                    
                    if ~front_veh_No  % 如果本lane内没找到前车
                        if  ~canReachNextEdges(vNo)  % 如果到不了下一个边，就设置虚拟车
                                front_veh_dist = remDist-10; % 比停止线更靠前
                                front_veh_No = 666; % 虚拟车号码
                                % front_veh_spd = 0; % 直接默认啦，不用再赋值

                        elseif nearJunction % 如果也没有虚拟车，如果本edge剩余距离小于定值，就检查下一个edge:交叉口内部edge
                        % 先找出在同一个交叉口内部，同样都是nearJunction的车车
                        checkList = find(opsStates & nearJunctions); % 在交叉口附近的车
                        sameJunctionList = findSameJunctionList(vNo,checkList,junctionIDs,currentJunctionID);
                        end
                    end
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                case 2 % 对其来说快要结束的边内，只需要考虑本edge内的车辆就好
                    % 所以不需要多写什么啦
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                case 3 % 如果在交叉口内部，就需要考虑
                    % 先检查本条edge内正常的车（检查过啦！）
                    % 靠交互检查
                    if front_veh_No  % 如果本lane内找到了前车，对其来说我确实是后车，不过我还可能通过交互找到更新的前车
                        % 先找出在同一个交叉口内部，nearJunction的车车
                        checkList = find(opsStates & nearJunctions); % 在交叉口附近的车
                        sameJunctionList = findSameJunctionList(vNo,checkList,junctionIDs,currentJunctionID,front_veh_No);
                    else 
                        % 先找出在同一个交叉口内部，nearJunction的车车
                        checkList = find(opsStates & nearJunctions); % 在交叉口附近的车
                        sameJunctionList = findSameJunctionList(vNo,checkList,junctionIDs,currentJunctionID);
                    end


            end

            [front_veh_dist, front_veh_No, front_veh_spd] = inJunctionInteraction(vNo,front_veh_dist, ...
                front_veh_No, front_veh_spd,spds,sameJunctionList,politenss,xs,ys,dirs,vehicleDriveLine,'direct');
            
            surroundVehMat(bidx,2) = front_veh_No; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
            surrVehDistMat(bidx,2) = front_veh_dist;
            if front_veh_No
                surrVehSpdMat(bidx,2) = front_veh_spd;
            else
                surrVehSpdMat(bidx,2) = spdLims(vNo);
            end

            %#######################################################################################
            % (4)寻找后车
            backVehNo = surroundVehMat(bidx+1,2);
            if backVehNo < 0 % 如果没有被其他流程检查出来过
                surroundVehMat(bidx+1,2) = 0; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
                % surrVehDistList_mat(bidx+1,2) = front_veh_dist;
                surrVehSpdMat(bidx+1,2) = 0;
            end

            %#######################################################################################
            
            % 首先判断一下左边是否可以换道
            if round(laneIdxDev) >= round(maxLaneIdxDevs(vNo)-0.5) || nearJunctionCanNotLC % 超出了左边的界限，无法再向左换道
                surroundVehMat(bidx,1) = 999; % 无法换道
                surrVehDistMat(bidx,1) = 1; % 给一个小值
                surrVehSpdMat(bidx,1) = 0;
                surroundVehMat(bidx+1,1) = 999; % 无法换道
                surrVehDistMat(bidx+1,1) = 1; % 给一个小值
                surrVehSpdMat(bidx+1,1) = 0;
            end
            % 再判断一下右边是否可以换道
            if round(laneIdxDev) <= 0 || nearJunctionCanNotLC % 超出了左边的界限，无法再向左换道
                surroundVehMat(bidx,3) = 999; % 无法换道
                surrVehDistMat(bidx,3) = 1; % 给一个小值
                surrVehSpdMat(bidx,3) = 0;
                surroundVehMat(bidx+1,3) = 999; % 无法换道
                surrVehDistMat(bidx+1,3) = 1; % 给一个小值
                surrVehSpdMat(bidx+1,3) = 0;
            end

            left_front_veh_dist = dftDist; % 默认距离
            left_front_veh_No = 0;
            left_front_veh_spd = 0;
            right_front_veh_dist = dftDist; % 默认距离
            right_front_veh_No = 0;
            right_front_veh_spd = 0;

            nextEdgeID = obj.getNextRouteEdgeID(vNo); % 获取导航路径上下一个边

            % (2)寻找左前车（同时找相对车的右后车）
            if surroundVehMat(bidx,1)<0 % 如果没有被其他流程检查出来过
                % 此时能到这里说明左边可以换道
                % 首先检查同一个edge内的左边车道的车辆
                leftLaneList = find(sameEdgeLogical & ...
                   laneNos == (laneNo + 1) & opsStates); % 这里左侧车道的所有未停止的车，肯定不包含自身

                for i = 1:length(leftLaneList) % 对每辆车进行检查
                    otherVNo = leftLaneList(i);
                    l_fv_dist = laneDists(otherVNo) - laneDists(vNo);
                    if l_fv_dist > 0 % 说明这辆车在前面
                        if l_fv_dist <= left_front_veh_dist
                            left_front_veh_dist = l_fv_dist;
                            left_front_veh_No = otherVNo;
                            left_front_veh_spd = spds(otherVNo);
                        end
                    end
                end

                if left_front_veh_No % 如果已经在左边的车道找到了左前车，那么对其来说我就是右后车啦
                    surroundVehMat(left_front_veh_No*2,3) = vNo;
                    surrVehDistMat(left_front_veh_No*2,3) = left_front_veh_dist;
                    surrVehSpdMat(left_front_veh_No*2,3) = spds(vNo);
                    
                % 如果没检查出来，再检查虚拟车
                elseif opsState ~= 2 % 如果不在自己路线的最终边上，那么nextEdgeID也不应该为[]
                    % disp(nextEdgeID); % 如果有问题可以检查一下是否是空的
                    leftLaneID = [currentEdgeID '_' char('0'+laneNo + 1)];
                    canReachNextEdge_left = obj.net.canReach_L2E(leftLaneID,nextEdgeID);
                    if ~canReachNextEdge_left % 如果左车道到达不了下一个edge
                        left_front_veh_dist = remDist-10; % 比停止线更靠前
                        left_front_veh_No = 666; % 虚拟车号码
                        % left_front_veh_spd = 0;

                    elseif nearJunction % 如果上面都不符合，看是否临近下一个路口，如果是，就用偏移，检查投影
                        % 找到相关车了，开始交互啦------------------------------------------------
                        [left_front_veh_dist, left_front_veh_No, left_front_veh_spd] = inJunctionInteraction(vNo,left_front_veh_dist, ...
                        left_front_veh_No, left_front_veh_spd,spds,sameJunctionList,politenss,xs,ys,dirs,vehicleDriveLine,'left');
                    end
                end

                surroundVehMat(bidx,1) = left_front_veh_No; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
                surrVehDistMat(bidx,1) = left_front_veh_dist;
                if left_front_veh_No
                    surrVehSpdMat(bidx,1) = left_front_veh_spd;
                else
                    surrVehSpdMat(bidx,1) = states(vNo,obj.var_spdLim);
                end
            end

            %#######################################################################################
            % (6)寻找右后车
            if surroundVehMat(bidx+1,3)<0 % 如果没有被其他流程检查出来过
                surroundVehMat(bidx+1,3) = 0; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
                % surrVehDistList_mat(bidx+1,3) = 1;
                surrVehSpdMat(bidx+1,3) = 0;
            end

            %#######################################################################################
            % (3)寻找右前车（同时找相对车的左后车）
            if surroundVehMat(bidx,3)<0 % 如果没有被其他流程检查出来过
                % 此时能到这里说明右边可以换道
                rightLaneList = find(sameEdgeLogical & ...
                   laneNos == (laneNo - 1) & opsStates); % 这里右侧车道的所有车未停止的车

                for i = 1:length(rightLaneList) % 对每辆车进行检查
                    otherVNo = rightLaneList(i);
                    r_fv_dist = laneDists(otherVNo) - laneDists(vNo);
                    if r_fv_dist > 0 % 说明这辆车在前面
                        if r_fv_dist <= right_front_veh_dist
                            right_front_veh_dist = r_fv_dist;
                            right_front_veh_No = otherVNo;
                            right_front_veh_spd = spds(otherVNo);
                        end
                    end
                end

                if right_front_veh_No % 如果已经在右边的车道找到了右前车，那么对其来说我就是左后车啦
                    surroundVehMat(right_front_veh_No*2,1) = vNo;
                    surrVehDistMat(right_front_veh_No*2,1) = right_front_veh_dist;
                    surrVehSpdMat(right_front_veh_No*2,1) = spds(vNo);
                    
                % 如果没检查出来，再检查虚拟车
                elseif opsState ~= 2 % 如果不在自己路线的最终边上，那么nextEdgeID也不应该为[]
                    % disp(nextEdgeID); % 如果有问题可以检查一下是否是空的
                    rightLaneID = [currentEdgeID '_' char('0'+laneNo - 1)];
                    canReachNextEdge_right = obj.net.canReach_L2E(rightLaneID,nextEdgeID);
                    if ~canReachNextEdge_right % 如果右车道到达不了下一个edge
                        right_front_veh_dist = remDist-10; % 比停止线更靠前
                        right_front_veh_No = 666; % 虚拟车号码
                        % right_front_veh_spd = 0;

                    elseif nearJunction % 如果上面都不符合，看是否临近下一个路口，如果是，就用偏移，检查投影
                        [right_front_veh_dist, right_front_veh_No, right_front_veh_spd] = inJunctionInteraction(vNo,right_front_veh_dist, ...
                        right_front_veh_No,right_front_veh_spd,spds,sameJunctionList,politenss,xs,ys,dirs,vehicleDriveLine,'right');
                    end
                end

                surroundVehMat(bidx,3) = right_front_veh_No; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
                surrVehDistMat(bidx,3) = right_front_veh_dist;
                if right_front_veh_No
                    surrVehSpdMat(bidx,3) = right_front_veh_spd;
                else
                    surrVehSpdMat(bidx,3) = states(vNo,obj.var_spdLim);
                end
            end


            %#######################################################################################
            % (5)寻找左后车
            if surroundVehMat(bidx+1,1)<0 % 如果没有被其他流程检查出来过
                surroundVehMat(bidx+1,1) = 0; % 如果检查了就是没有，那就大大方方写0,否则就是有号码
                % surrVehDistList_mat(bidx+1,1) = 1;
                surrVehSpdMat(bidx+1,1) = 0;
            end
                                
        end
    end
  
end


function sameJunctionList = findSameJunctionList(vNo,checkList,junctionIDs,currentJunctionID,front_veh_No)
    if nargin == 5
        num2check = 2; % 只有当前车和本edge内的前车%%%%%
    else
        num2check = 1; % 只有当前车
    end

    if length(checkList) == num2check 
        sameJunctionList = []; % 就没有交叉路口内的交互车啦
    else
        sameJunctionList = checkList;
        valid_count = 0;
        for i = 1:length(checkList)
            otherVNo = checkList(i);
            if nargin == 5
                if (otherVNo ~= vNo) && (otherVNo ~= front_veh_No) && strcmp(junctionIDs{otherVNo},currentJunctionID) 
                    % 如果不是自己,且是在同一路口
                    valid_count = valid_count + 1;
                    sameJunctionList(valid_count) = otherVNo;
                end
            else
                if (otherVNo ~= vNo) && strcmp(junctionIDs{otherVNo},currentJunctionID) 
                    % 如果不是自己,且是在同一路口
                    valid_count = valid_count + 1;
                    sameJunctionList(valid_count) = otherVNo;
                end
            end
        end
        sameJunctionList(valid_count+1:end) = [];
    end

end


function [veh_dist, veh_No, veh_spd] = inJunctionInteraction(vNo,veh_dist,veh_No,veh_spd,spds, ...
                                            sameJunctionList,politenss,xs,ys,dirs,vehDriveLine,direction)
    if ~isempty(sameJunctionList) % 如果存在周车
        for i = 1:length(sameJunctionList) % 对每个筛选出来的周车进行检查
            otherVNo = sameJunctionList(i);
            % 需要对下面vNo的线进行offset
            switch direction
                case 'left'
                    driveLine = offset2DCurve_dir_mex(vehDriveLine{vNo},3.2,'left'); % 这里车道宽度默认为3.2了
                case 'right'
                    driveLine = offset2DCurve_dir_mex(vehDriveLine{vNo},3.2,'right'); % 这里车道宽度默认为3.2了
                otherwise
                    driveLine = vehDriveLine{vNo};
            end
            [isIntersect, ~, v_dist, ov_dist] = findIntersection_mex(driveLine, vehDriveLine{otherVNo});
            if isIntersect && v_dist < veh_dist % 如果我的轨迹和这辆车的轨迹相交,且比原有左前车更近（或者比默认值更近）
                v_timeHeadWay = v_dist / spds(vNo);
                ov_timeHeadWay = ov_dist / spds(otherVNo);
                if abs(v_timeHeadWay - ov_timeHeadWay) < 2 %如果小于2s的时距，否则互相不交互
                    if dirs(vNo) > dirs(otherVNo) % 本车拥有更高通行权,就只考虑闯入本车行驶范围内的低通行权车
                        [~, dev, lineDist, ~] = projPoint2Polyline_mex(driveLine,[xs(otherVNo),ys(otherVNo)]);
                        if dev < 3.5 && lineDist < veh_dist  % 小于车道
                            veh_No = otherVNo;
                            veh_dist = lineDist;
                            veh_spd = spds(otherVNo);
                        end
                    elseif dirs(vNo) == dirs(otherVNo) % 两车通行权相同，用politeess决胜负
                        if politenss(vNo) > politenss(otherVNo) % 我更礼貌
                            veh_dist = v_dist;
                            veh_No = otherVNo; 
                            veh_spd = spds(otherVNo);
                        end
                    else % 本车通行权低，应该让行
                        veh_dist = v_dist;
                        veh_No = otherVNo; 
                        veh_spd = spds(otherVNo);
                    end
                end
            end
        end
    end %---------------------------------------------------------------------

end