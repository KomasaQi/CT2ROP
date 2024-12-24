function fromLaneList = getFromLaneList(lane_to_connection_dict,laneID)
    if isKey(lane_to_connection_dict,laneID)
        connection_num = length(lane_to_connection_dict{laneID});
        fromLaneList = cell(connection_num,1);
        for i = 1:connection_num
            fromLaneList{i} = lane_to_connection_dict{laneID}{i}.from;
        end
    else
        error('字典中不存在此键！正确的调用格式是：fromLaneList = getFromLaneList(lane_to_connection_dict,laneID)')
    end
end