function [routeRelation,remainDist]=getNextRoutePointInfo(entity_dict,connection_dict,globalRoute,laneID,LaneDist)
    persistent lastRouteIndex
    if isempty(lastRouteIndex)
        lastRouteIndex = 1;
    end
    edgeID = entity_dict{laneID}.getEdgeID();
    current_idx = find(strcmp(globalRoute, edgeID));
    if isempty(current_idx)
        current_idx = lastRouteIndex;
        remainDist = 0;
    else
        lastRouteIndex = current_idx;
        laneLength = max(entity_dict{laneID}.length,1e-2);
        remainDist = laneLength - LaneDist;
    end


    if current_idx >= length(globalRoute)
        routeRelation = 'g'; %almost goal 即将到达目的地
    else
        nextEdgeID = globalRoute{current_idx+1};
        routeRelation = 'n'; %unknown
        for i = 1:connection_dict{edgeID}.connection_num
            theConnection = connection_dict{edgeID}.connections{i};
            if strcmp(theConnection.to,nextEdgeID)
                dir = theConnection.dir;
                if strcmpi(dir,'r')
                    routeRelation = 'r';
                elseif strcmpi(dir,'l')
                    routeRelation = 'l';
                elseif strcmpi(dir,'s')
                    routeRelation = 's';
                elseif strcmpi(dir,'t')
                    routeRelation = 't';
                else
                    error(['dir: ' dir '不是预先定义的类型呜呜，主人快检查一下！'])
                end
            end 
        end
    end


end