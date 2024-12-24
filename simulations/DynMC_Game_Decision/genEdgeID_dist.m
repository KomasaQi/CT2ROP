% 用于生成用来从simRoadNetwork_dict中查询仿真用小地图本车当前edge和lanePosition时的一个id

function edgeID_dist = genEdgeID_dist(edgeID,dist,resolution)
    edgeID_dist = [edgeID '[@' num2str(resolution*round(dist/resolution)) ']'];
end