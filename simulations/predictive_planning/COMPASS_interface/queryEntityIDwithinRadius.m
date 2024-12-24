%{
    查找给定的point周围radius半径的所有junction的ID
%}
function [edgeList,junctionList] = queryEntityIDwithinRadius(proxyMat,point,radius) %#codegen
    j_distances = sqrt((proxyMat.xyMat(:,1)-point(1)).^2+(proxyMat.xyMat(:,2)-point(2)).^2);
    junctionIdxs = find(j_distances < radius);
    junctionList = {proxyMat.dict{junctionIdxs'}};

    e_distances1 = sqrt((proxyMat.edgeEnd_xyMat(:,1)-point(1)).^2+(proxyMat.edgeEnd_xyMat(:,2)-point(2)).^2);
    e_distances2 = sqrt((proxyMat.edgeEnd_xyMat(:,3)-point(1)).^2+(proxyMat.edgeEnd_xyMat(:,4)-point(2)).^2);
    edgeIdxs = find(min([e_distances1,e_distances2],[],2) < radius);
    edgeList = {proxyMat.edge_dict{edgeIdxs'}};

end