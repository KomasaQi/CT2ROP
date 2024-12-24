function [heading,heading_cos_sin] = getVehicleHeading(entity_dict,laneID,dist)
    if ~isKey(entity_dict,laneID)
        error(['输入的laneID' laneID '不在实体字典中，主人快检查一下喵'])
    else
        shape = entity_dict{laneID}.shape;
        distance = xy2dist(shape);
        idx = findPosition(distance, dist);
        if idx == length(distance)
            heading_cos_sin = [];
            heading = [];
        else
            heading_cos_sin = shape(idx+1,:)-shape(idx,:);
            heading_cos_sin = heading_cos_sin/norm(heading_cos_sin);
            heading = atan2(heading_cos_sin(2),heading_cos_sin(1));
        end
    end
end


function index = findPosition(arr, target)
    % arr: 递增数列
    % target: 目标数字
    % index: 目标数字所在的区间起始索引
    
    left = 1;
    right = length(arr);

    while left <= right
        mid = floor((left + right) / 2);

        if arr(mid) == target
            index = mid; % 如果目标数字恰好在数列中，返回其索引
            return;
        elseif arr(mid) < target
            left = mid + 1; % 目标数字在右侧
        else
            right = mid - 1; % 目标数字在左侧
        end
    end

    % 如果没有找到恰好等于目标数字的值，返回区间
    index = right; % 目标数字位于 right 和 right+1 之间
end