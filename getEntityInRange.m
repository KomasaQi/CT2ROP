% 相比于queryEntityIDwithinRadius()，多了edgeID这个考虑
function [edgeList, junctionList] = getEntityInRange(proxyMat,point,radius,edgeID)
    [edgeList,junctionList] = queryEntityIDwithinRadius(proxyMat,point,radius);
    edgeList = unique([edgeList,{edgeID}]);
end