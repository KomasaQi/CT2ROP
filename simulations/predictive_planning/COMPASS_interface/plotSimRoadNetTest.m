figNum = 3;

keys = simRoadNetwork_dict.keys;
for i = 1:5:length(keys)
    key = keys{i};
    figNum = figNum + 1;
    figure(figNum)
    plotEdgesCenterLines(new_entity_dict,simRoadNetwork_dict{key},figNum)
    hold off
end


function plotEdgesCenterLines(new_entity_dict,edgeList,FigureID)
    figure(FigureID)
    hold on
    for i = 1:length(edgeList)
        for j = 1:new_entity_dict{edgeList{i}}.laneNum
            shape = new_entity_dict{new_entity_dict{edgeList{i}}.getLaneID(j-1)}.shape;
            speed = new_entity_dict{new_entity_dict{edgeList{i}}.getLaneID(j-1)}.speed;
            plot(shape(:,1),shape(:,2),'Color',[speed/33.3 0 1-speed/33.3],'LineWidth',0.5)
        end
    end
    axis equal
end