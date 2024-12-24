% 定义 TraCI 查询循环函数
function traciQueryLoop(dq)
    disp('11111')
    while true
        idList = traci.vehicle.getIDList();
        for i = 1:length(idList)
            vehicleID = idList{i};
            laneID = traci.vehicle.getLaneID(vehicleID);
            % 模拟查询结果
            result = laneID; % 示例查询结果
            key = vehicleID;
            
            % 将查询结果通过数据队列发送给主线程
            send(dq, {key, result});
            disp('发送数据啦')
        end
    end
end
