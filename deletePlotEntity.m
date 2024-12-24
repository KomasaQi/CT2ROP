function new_handle_dict = deletePlotEntity(entity_dict,entityList,handle_dict)
    new_handle_dict = handle_dict;
    if ~isempty(entityList)
        for i = 1:length(entityList)
            id = entityList{i};
            if isKey(handle_dict,id)
                delete(handle_dict(id))
                new_handle_dict(id) = [];
            elseif isa(entity_dict{id},'Edge_SUMO')
                for j = 1:entity_dict{id}.laneNum
                    laneID = entity_dict{id}.getLaneID(j-1);
                    if isKey(handle_dict,laneID)
                        delete(handle_dict(laneID))
                        new_handle_dict(laneID) = [];
                    end
                end
            end
        end
    end

end