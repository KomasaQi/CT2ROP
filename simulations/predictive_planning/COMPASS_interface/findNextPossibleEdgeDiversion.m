function [divertEdgeList,viaEdges,laneIDSeque] = findNextPossibleEdgeDiversion(entity_dict,connection_dict,edgeID)
    
    [nextEdgeID,laneIDSeque] = findPossibleNextEdge(entity_dict,connection_dict,edgeID);
    if isempty(nextEdgeID) % 当不存在下一个edge时，返回空的内容
        divertEdgeList = [];
        viaEdges = [];
        laneIDSeque = [];
    elseif length(nextEdgeID) >=2  % 当不只有一个下一个edge时，说明下一个edge已经分叉啦，满足条件了
        divertEdgeList = nextEdgeID;
        viaEdges = [];
    else % 当下一个edge只有一个时
        if entity_dict{edgeID}.laneNum ~= entity_dict{nextEdgeID{1}}.laneNum
            % 如果当前edge和下一个edge的车道数不一致，那也算是满足于要求啦
            divertEdgeList = nextEdgeID;
            viaEdges = [];
        else % 本edge和下一个edge车道数全相同，说明没有还没有意图分叉的可能性，要继续往后探索
            viaEdges = nextEdgeID;
            [divertEdgeList,viaEdgesNext,laneIDSeque] = ...
            findNextPossibleEdgeDiversion(entity_dict,connection_dict,nextEdgeID{1});
            if ~isempty(viaEdgesNext)
                viaEdges = [viaEdges;viaEdgesNext];
            else

            end
        end
        
    end


end