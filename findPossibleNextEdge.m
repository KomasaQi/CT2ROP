%% findPossibleNextEdge 查找可能的下一个edge和lane
% 用途：设计用于周车的意图估计之前进行可能性筛查
function [nextEdgeID,laneIDSeque] = findPossibleNextEdge(entity_dict,connection_dict,edgeID)
    if ~isKey(entity_dict,edgeID)
        disp(['edge' edgeID '不在实体字典中呜呜，请主人检查一下！（超凶）'])
        nextEdgeID = [];
        laneIDSeque = [];
    elseif ~isKey(connection_dict,edgeID)
        nextEdgeID = [];
        laneIDSeque = [];
    else
        nextEdgeID = cell(1,connection_dict{edgeID}.connection_num);
        laneIDSeque = cell(2,connection_dict{edgeID}.connection_num);
        for i = 1:length(nextEdgeID)
            if isempty(connection_dict{edgeID}.connections{i}.via)
                nextEdgeID{i} = connection_dict{edgeID}.connections{i}.to;
                laneIDSeque{1,i} = [edgeID '_' connection_dict{edgeID}.connections{i}.fromLane];
                laneIDSeque{2,i} = [nextEdgeID{i} '_' num2str(connection_dict{edgeID}.connections{i}.toLane)];

            else
                nextEdgeID{i} = entity_dict{connection_dict{edgeID}.connections{i}.via}.getEdgeID();
                laneIDSeque{1,i} = [edgeID '_' num2str(connection_dict{edgeID}.connections{i}.fromLane)];
                laneIDSeque{2,i} = connection_dict{edgeID}.connections{i}.via;

            end
        end
        nextEdgeID = unique(nextEdgeID);
    end
    

end