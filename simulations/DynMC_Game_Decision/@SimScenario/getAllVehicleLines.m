%--------------------------------------------------------------------------------------------------------------
%------------------------------------------场景状态：vehDriveLine------------------全部更新----------------------
%-------------------------------------------------------------------------------------------------------------
% 对每辆车，找出每一辆车的当前车道下向前foreseeTime秒匀速行驶距离的车道中心线
function vehDriveLine_init = getAllVehicleLines(obj)
    vehDriveLine_init = cell(obj.vehNum,1);
    lineLengths = max((obj.vehState(:,obj.var_spd) + 1)*obj.foreseeTime,15); % 保证这条线至少有一定的长度
    laneDists = obj.vehState(:,obj.var_laneDist);
    opsStates = obj.vehState(:,obj.var_opsState);
    for vNo = 1:obj.vehNum
        if opsStates(vNo) % 如果还没结束，就计算，否则就不浪费算力啦
            % 首先是计算需要多长的centerLine
            lineLength = lineLengths(vNo);
            currentLaneID = obj.laneIDs{vNo};
            laneDist = laneDists(vNo);
            currentLaneShape = obj.net.getLaneShape(currentLaneID);
            if ~obj.vehState(vNo,obj.var_canReachNextEdge) % 如果在当前车道还无法到达下一个edge，就只生成本edge内的
                fullLine = currentLaneShape;
            else % 如果可以到达，就先提取前面的2个edge
                % routeNum = obj.vehicles{vNo}.routeNum;
                if strncmp(currentLaneID,':',1) % 如果是在交叉口内部的边
                    % 那么当前的routeIdx对应的是已经行驶过的边，且一定存在下一个正常边
                    % 且对于轿车口内的车道来说，可以到达的下一个车道是唯一的
                    toLaneID = obj.net.lc_from_dict{currentLaneID}{1}.to;
                    line1 = currentLaneShape;
                    line2 = obj.net.getLaneShape(toLaneID);
                else % 也没有马上结束，那么就是还有下一个边，以及下一个边的中间边
                    % 需要找到从本车道到下一个edge可能的lane连接，并选取其中最近的lane作为目标lane
                    line1 = currentLaneShape;
                    dist_min = inf;
                    idx = 0;
                    lc_connections = obj.net.get_lc_from_connections(currentLaneID);
                    for i = 1:length(lc_connections)
                        toLaneID_temp = lc_connections{i}.to;
                        nextEdgeID = obj.getNextRouteEdgeID(vNo);
                        if strcmp(toLaneID_temp(1:end-2),nextEdgeID) % 需要保证这个连接的下一个edge是route中的下一个edge
                            laneShape = obj.net.getLaneShape(toLaneID_temp);
                            lanePos = laneShape(1,:);
                            dist_xy = norm(lanePos - line1(end,:));
                            if dist_xy < dist_min
                                idx = i;
                            end
                        end
                    end
                    if ~idx
                        error(['vNo:' num2str(vNo) '无法找到与route中下一个目标edge内lane的连接,currentlaneID=' currentLaneID]);
                    end
                    toLaneID = lc_connections{idx}.via;
                    line2 = obj.net.getLaneShape(toLaneID);
                end
                % toLaneIDs{vNo} = toLaneID;
                fullLine = [line1;line2(2:end,:)];
            end
            vehDriveLine_init{vNo} = getVehLineInDist(fullLine,laneDist,lineLength);
        end
    end
end