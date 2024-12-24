function simRoadNetwork_dict = genSimRoadNetworkDict(allDict,egoID)
    s_front = 1000;
    s_back = 400;
    resolution = 100; % 精度为每50m生成一次RoadStruct_dict
    globalRoute = traci.vehicle.getRoute(egoID);
    routeEdgeNum = length(globalRoute);
    eachLens = zeros(routeEdgeNum,1);
    for i = 1:routeEdgeNum
        edgeID = globalRoute{i};
        edgeLen = 0;
        for j = 1:allDict.entity_dict{edgeID}.laneNum % 得到edge的最长车道长度
            laneID = allDict.entity_dict{edgeID}.getLaneID(j-1);
            laneLen = allDict.entity_dict{laneID}.length; 
            if edgeLen < laneLen
                edgeLen = laneLen;
            end
        end
        eachLens(i) = edgeLen;
    end
    % eachDists = cumsum(eachLens);
    simRoadNetwork_dict = dictionary(string([]),{});% SimRoadNetwork()
    for i = 1:routeEdgeNum
        edgeID = globalRoute{i};
        edgeLen = eachLens(i);
        edgeDists = linspace(0,edgeLen,round(edgeLen/10+2)); % 将edge划分为大约每10m一份
        for j = 1:length(edgeDists)
            edgeDist = edgeDists(j);
            edgeID_dist = [edgeID '[@' num2str(resolution*round(edgeDist/resolution)) ']'];
            if isKey(simRoadNetwork_dict,edgeID_dist)
                continue
            else
                % simRoadStruct_dict(edgeID_dist) = SimRoadNetwork();
                %% 前向寻找edge
                cumRemDist = -edgeDist;
                frontEdgeList = {edgeID};
                for edgeNo = i:routeEdgeNum
                    cumRemDist =  cumRemDist + eachLens(edgeNo);
                    currentEdgeID = globalRoute{edgeNo};
                    nextJunctionID = allDict.entity_dict{currentEdgeID}.to;
                    tempEdgeList = findLv1EdgesList(allDict.entity_dict,allDict.connection_from_dict,nextJunctionID);
                    frontEdgeList = unique([tempEdgeList;frontEdgeList]);
                    if cumRemDist > s_front
                        break
                    end
                end
                %% 后向寻找edge
                remEdgeDist = edgeLen - edgeDist;
                cumPastDist = -remEdgeDist;
                backEdgeList = {};
                for edgeNo = i:-1:1
                    cumPastDist =  cumPastDist + eachLens(edgeNo);
                    currentEdgeID = globalRoute{edgeNo};
                    lastJunctionID = allDict.entity_dict{currentEdgeID}.from;
                    tempEdgeList = findLv1EdgesList(allDict.entity_dict,allDict.connection_from_dict,lastJunctionID);
                    backEdgeList = unique([tempEdgeList;backEdgeList]);
                    if cumPastDist > s_back
                        break
                    end
                end
                sceneEdgeList = unique([frontEdgeList;backEdgeList]);
                
                sceneJunctionList = cell(length(sceneEdgeList),1);
                junctionCounter = 0;
                for k = 1:length(sceneEdgeList)
                    sceneEdgeID = sceneEdgeList{k};
                    if strncmp(sceneEdgeID,':',1)
                        junctionCounter = junctionCounter + 1;
                        lastUnderscoreIdx = find(sceneEdgeID=='_',1,'last'); 
                        sceneJunctionList{junctionCounter} = sceneEdgeID(2:lastUnderscoreIdx-1);
                    end
                    
                end
                sceneJunctionList(junctionCounter+1:end) = [];
                sceneJunctionList = unique(sceneJunctionList);
                frindgeEdgeArray = ones(length(sceneEdgeList),1);
                edge_No_dict = dictionary(string([]),[]);

                e_dict = dictionary(string([]),{});
                c_from_dict = dictionary(string([]),{});
                c_to_dict = dictionary(string([]),{});
                lc_from_dict = dictionary(string([]),{});
                lc_to_dict = dictionary(string([]),{});

                for k = 1:length(sceneEdgeList)
                    sceneEdgeID = sceneEdgeList{k};
                    fromJunctionID = getFromJunctionID(allDict.entity_dict,sceneEdgeID);
                    toJunctionID = getToJunctionID(allDict.entity_dict,sceneEdgeID);
                    if ismember(fromJunctionID,sceneJunctionList) && ismember(toJunctionID,sceneJunctionList)
                        frindgeEdgeArray(k) = 0;
                    end
                    edge_No_dict(sceneEdgeID) = k;

                    e_dict{sceneEdgeID} = allDict.entity_dict{sceneEdgeID};
                    for p = 1:allDict.entity_dict{sceneEdgeID}.laneNum
                        sceneEdge_laneID = allDict.entity_dict{sceneEdgeID}.getLaneID(p-1);
                        e_dict{sceneEdge_laneID} = allDict.entity_dict{sceneEdge_laneID};
                        if isKey(allDict.lane_from_connection_dict,sceneEdge_laneID)
                            lc_from_dict{sceneEdge_laneID} = allDict.lane_from_connection_dict{sceneEdge_laneID};
                        end
                        if isKey(allDict.lane_to_connection_dict,sceneEdge_laneID)
                            lc_to_dict{sceneEdge_laneID} = allDict.lane_to_connection_dict{sceneEdge_laneID};
                        end
    
                    end
                    if isKey(allDict.connection_from_dict,sceneEdgeID)
                        c_from_dict{sceneEdgeID} = allDict.connection_from_dict{sceneEdgeID};
                    end
                    if isKey(allDict.connection_to_dict,sceneEdgeID)
                        c_to_dict{sceneEdgeID} = allDict.connection_to_dict{sceneEdgeID};
                    end
                    
                end
                
                for k = 1:length(sceneJunctionList)
                    sceneJunctionID = sceneJunctionList{k};
                    e_dict{sceneJunctionID} = allDict.entity_dict{sceneJunctionID};
                end
                


                simRoadNetwork_dict{edgeID_dist} = SimRoadNetwork('edgeList',sceneEdgeList, ...
                    'junctionList',sceneJunctionList,'frindgeEdgeArray',frindgeEdgeArray, ...
                    'edge_No_dict',edge_No_dict,'e_dict',e_dict,'c_from_dict',c_from_dict, ...
                    'c_to_dict',c_to_dict,'lc_from_dict',lc_from_dict,'lc_to_dict',lc_to_dict);



            end
        end


    end
end

function edgeList = findLv1EdgesList(entity_dict,connection_dict,JunctionID)
% nextJunctionID = entity_dict{edgeID}.to;
incLanes_NJ = entity_dict{JunctionID}.incLanes;
validEdgesNum = 0;
edgeList = cell(100,1);
for i = 1:length(incLanes_NJ)
    validEdgesNum = validEdgesNum + 1;
    edgeList(validEdgesNum)=getEdgeID(incLanes_NJ(i));
end
for i = 1:length(incLanes_NJ)
    incEdgeID = getEdgeID(incLanes_NJ{i});
    for j = 1:connection_dict{incEdgeID}.connection_num
        validEdgesNum = validEdgesNum + 1;
        edgeList{validEdgesNum} = connection_dict{incEdgeID}.connections{j}.to;
        validEdgesNum = validEdgesNum + 1;
        edgeList{validEdgesNum} = getEdgeID(connection_dict{incEdgeID}.connections{j}.via);
    end
end
edgeList = unique(edgeList(1:validEdgesNum));
% validEdgesNum = length(sceneEdgesID);

end


function edgeID = getEdgeID(laneID)
    edgeID = regexprep(laneID, '_\d+$', '');
end

