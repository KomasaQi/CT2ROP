% 如果没载入数据，记得载入一下
% load ProcessedMap_yizhuang.mat  
edgeID = '30048339#25';% 一个快进入交叉路口的长边

figureID = 1;

[divertEdgeList,viaEdges,laneIDSeque] =...
    findNextPossibleEdegeDiversion(entity_dict,connection_dict,edgeID);

plotSUMOentity(entity_dict,connection_dict,{edgeID},figureID)
plotSUMOentity(entity_dict,connection_dict,viaEdges,figureID)
plotSUMOentity(entity_dict,connection_dict,divertEdgeList,figureID)