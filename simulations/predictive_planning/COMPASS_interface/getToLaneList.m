function toLaneList = getToLaneList(lane_from_connection_dict,laneID)
    if isKey(lane_from_connection_dict,laneID)
        connection_num = length(lane_from_connection_dict{laneID});
        toLaneList = cell(connection_num,1);
        for i = 1:connection_num
            toLaneList{i} = lane_from_connection_dict{laneID}{i}.to;
        end
    else
        error('字典中不存在此键！正确的调用格式是：toLaneList = getToLaneList(lane_from_connection_dict,laneID)')
    end
end