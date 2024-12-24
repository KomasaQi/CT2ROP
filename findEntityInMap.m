function entity = findEntityInMap(map,EntityID)

    entity = [];
    for i = 1:length(map.Children)
        id = getAttribute(map.Children(i),'id');
        if ~isempty(id)
            if strcmp(id,EntityID)
                entity = map.Children(i);
                break
            else
                for j = 1:length(map.Children(i).Children)
                    id0 = getAttribute(map.Children(i).Children(j),'id');
                    if strcmp(id0,EntityID)
                        entity = map.Children(i).Children(j);
                        break
                    end
                    
                end
            end
            
        end

    end


end