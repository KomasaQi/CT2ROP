function [divertEdgeList,viaEdges,laneIDSeque] = findLastPossibleEdgeDiversion(entity_dict,connection_dict,edgeID)
    
    [lastEdgeID,laneIDSeque] = findPossibleLastEdge(entity_dict,connection_dict,edgeID);
    if isempty(lastEdgeID) % 当不存在上一个edge时，返回空的内容
        divertEdgeList = [];
        viaEdges = [];
        laneIDSeque = [];
    elseif length(lastEdgeID) >=2  % 当不只有一个上一个edge时，说明上一个edge已经分叉啦，满足条件了
        divertEdgeList = lastEdgeID;
        viaEdges = [];
    else % 当上一个edge只有一个时
        if entity_dict{edgeID}.laneNum ~= entity_dict{lastEdgeID{1}}.laneNum
            % 如果当前edge和上一个edge的车道数不一致，那也算是满足于要求啦
            divertEdgeList = lastEdgeID;
            viaEdges = [];
        else % 本edge和上一个edge车道数全相同，说明没有还没有意图分叉的可能性，要继续往后探索
            viaEdges = lastEdgeID;
            [divertEdgeList,viaEdgesLast,laneIDSeque] = ...
            findLastPossibleEdgeDiversion(entity_dict,connection_dict,lastEdgeID{1});
            if ~isempty(viaEdgesLast)
                viaEdges = [viaEdges;viaEdgesLast];
            else

            end
        end
        
    end


end