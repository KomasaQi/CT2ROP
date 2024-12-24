%{
用于从地图结构体中读取生成一个结构体proxyMat，其中mat为邻接关系矩阵
junctionIdx_dict用于存储这个矩阵的索引与junctionID的一一映射关系方便快速查找
%}
function proxyMat = genProxyMapFromMap(map,entity_dict)
    proxyMat = struct('dict',dictionary(),'xyMat',zeros(length(map.Children),2),...
        'edge_dict',dictionary(),'edgeEnd_xyMat',zeros(length(map.Children),4));
    junction_counter = 0;
    edge_counter = 0;
    for i = 1:length(map.Children)
        theChild = map.Children(i);
        if strcmp(theChild.Name,'junction')
            id = getAttribute(theChild,'id');
            if ~strncmp(id,':',1)
                junction_counter = junction_counter + 1;
                proxyMat.dict(junction_counter) = {id};
                proxyMat.xyMat(junction_counter,:) = ...
                    [str2double(getAttribute(theChild,'x')),...
                    str2double(getAttribute(theChild,'y'))];
            end
        elseif strcmp(theChild.Name,'edge')
            id = getAttribute(theChild,'id');
            if ~strncmp(id,':',1)
                edge_counter = edge_counter + 1;
                proxyMat.edge_dict(edge_counter) = {id};
                p1end = [0,0];
                p2end = [0,0];
                laneNum = entity_dict{id}.laneNum;
                for j = 1:laneNum
                    
                    p1end = p1end + entity_dict{entity_dict{id}.getLaneID(j-1)}.shape(1,:)/laneNum;
                    p2end = p2end + entity_dict{entity_dict{id}.getLaneID(j-1)}.shape(end,:)/laneNum;
                end
                proxyMat.edgeEnd_xyMat(edge_counter,:) = [p1end,p2end];
            end
        end
    end
    proxyMat.xyMat(junction_counter+1:end,:) = [];
    proxyMat.edgeEnd_xyMat(edge_counter+1:end,:) = [];

end