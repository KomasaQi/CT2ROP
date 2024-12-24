%% findPossibleLastEdge 查找可能的上一个edge
% 用途：设计用于周车的意图估计之前进行可能性筛查
function [lastEdgeID,laneIDSeque] = findPossibleLastEdge(entity_dict,connection_dict,edgeID)
    if ~isKey(entity_dict,edgeID)
        error(['edge' edgeID '不在实体字典中呜呜，请主人检查一下！（超凶）'])
    else
        laneIDSeque = cell(0);
        fromJunctionID = getFromJunctionID(entity_dict,edgeID);
        lastEdgeID = cell(0);
        if strncmp(edgeID,':',1)
            candidateLanes = entity_dict{fromJunctionID}.incLanes;
        else
            candidateLanes = entity_dict{fromJunctionID}.intLanes;
        end
        
        if ~isequal(candidateLanes,{''})
            lastEdgeID_candidate = fromLaneIDsgetEdgeIDs(entity_dict,candidateLanes);
            % 对于每个lastEdgeID验证其是否能到达edgeID
            for i = 1:length(lastEdgeID_candidate)
               lastID = lastEdgeID_candidate{i};
               keepFlag = false;
               for j = 1:connection_dict{lastID}.connection_num
                    connections = connection_dict{lastID}.connections{j};
                    if strncmp(edgeID,':',1)
                        if strcmp(edgeID,entity_dict{connection_dict{lastID}.connections{j}.via}.getEdgeID())
                            laneIDSeque([1 2],end+1)=...
[{entity_dict{connections.from}.getLaneID(connections.fromLane)};{connections.via}];
                            keepFlag = true;
                            
                        end
                    else
                        if strcmp(edgeID,connection_dict{lastID}.connections{j}.to)
                            laneIDSeque([1 2],end+1)=...
[{entity_dict{connections.from}.getLaneID(connections.fromLane)};...
 {entity_dict{connections.to}.getLaneID(connections.toLane)}];
                            keepFlag = true;
                            
                        end
                    end
               end
               if keepFlag % 如果能到达edgeID
                    lastEdgeID{end+1} = lastID;
               end

            end

        end
    end
    

end


function edgeIDs = fromLaneIDsgetEdgeIDs(entity_dict,laneIDs)
    edgeIDs = laneIDs;
    if ~isempty(laneIDs)
        for i = 1:length(laneIDs)
            edgeIDs{i} = entity_dict{laneIDs{i}}.getEdgeID();
        end
        edgeIDs = unique(edgeIDs);
    end

end