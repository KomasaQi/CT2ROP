function handle_dict = plot3DirectionArrow(dirArrowMap,figureID,idList,plotReq)
    handle_dict = dictionary();
    if ~isempty(idList)
        figure(figureID)
        hold on
        for i = 1:length(idList)
            id = idList{i};
            shape = dirArrowMap.dict{id}.shape;
            handle = fill3(shape(:,1),shape(:,2),plotReq.height*ones(length(shape),1),...
                plotReq.color,'FaceAlpha',plotReq.faceAlpha,'EdgeColor',plotReq.edgeColor,...
                'LineWidth',plotReq.lineWidth);
            handle_dict(id) = handle;
        end
    end

end