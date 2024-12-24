function [edgeList,junctionList] = plotNLevelRoadNet(entity_dict,connection_dict,edgeID,N,figureID,plotType,subProbType)
    edgeList = cell(0);
    if N ~= 0
        if nargin<7
            [divertEdgeListLast,viaEdgesLast,~] = ...
                findLastPossibleEdegeDiversion(entity_dict,connection_dict,edgeID);
            [divertEdgeListNext,viaEdgesNext,~] = ...
                findNextPossibleEdegeDiversion(entity_dict,connection_dict,edgeID);
            divertEdgeList = unique([divertEdgeListNext,divertEdgeListLast]);
            viaEdges = unique([viaEdgesLast;viaEdgesNext]);
            edgeList = unique([divertEdgeList';viaEdges]);
            if N > 1
                if ~isempty(divertEdgeList)
                    for i = 1:length(divertEdgeList)
                        subEdgeID = divertEdgeList{i};
                        edgeListForward = plotNLevelRoadNet(entity_dict,connection_dict,subEdgeID,N-1,figureID,'forward');
                        edgeListBackward = plotNLevelRoadNet(entity_dict,connection_dict,subEdgeID,N-1,figureID,'backward');
                        edgeList = unique([edgeList;edgeListForward;edgeListBackward]);
                    end
                end
            end
        else
            if strcmp(subProbType,'forward')
        [divertEdgeList,viaEdges,~] = ...
            findNextPossibleEdegeDiversion(entity_dict,connection_dict,edgeID);
        edgeList = unique([divertEdgeList';viaEdges]);
            elseif strcmp(subProbType,'backward')
        [divertEdgeList,viaEdges,~] = ...
            findLastPossibleEdegeDiversion(entity_dict,connection_dict,edgeID);
        edgeList = unique([divertEdgeList';viaEdges]);
            else
                error('subProbType不是设定的类型，主人快检查一下喵，啊~')
            end
            if N > 1
                if ~isempty(divertEdgeList)
                    for i = 1:length(divertEdgeList)
                        subEdgeID = divertEdgeList{i};
                        edgeListNext = plotNLevelRoadNet(entity_dict,connection_dict,subEdgeID,N-1,figureID,subProbType);
                        edgeList = unique([edgeList;edgeListNext]);
                    end
                end
            end
        end


    end
    if nargin<7
        edgeList = unique([edgeList;{edgeID}]);
        junctionList = cell(0);
        for i = 1:length(edgeList)
            theEdgeID = edgeList{i};
            junctionList = unique([junctionList;...
                {getFromJunctionID(entity_dict,theEdgeID)};...
                {getFromJunctionID(entity_dict,theEdgeID)}]);
        end
        if strncmp(plotType,'2',1)
            plotSUMOentity(entity_dict,connection_dict,edgeList,figureID)
            plotSUMOentity(entity_dict,connection_dict,junctionList,figureID)
        else
            plot3SUMOentity(entity_dict,edgeList,figureID)
            plot3SUMOentity(entity_dict,junctionList,figureID)
        end
    end
end