% % 创建共享的字典和数据队列
sharedDict = dictionary(string([]),{});
dq = parallel.pool.DataQueue;
% 
% % 创建用于查询 TraCI 信息的异步任务
f1 = parfeval(@traciQueryLoop, 0, dq);

% 主线程接收并更新共享字典
% afterEach(dq, @(data) updateSharedDict(data, sharedDict));

vehicleID = 'f_2.2';
for i = 1:5
    % if dq.QueueLength > 0
    %     % 从队列中获取数据
    %     [key, value] = receive(dq);
    % 
    %     % 更新共享字典
    %     sharedDict(key) = value;
    % end
    % tic
    % laneID = traci.vehicle.getLaneID(vehicleID);
    sharedDict
    % t = toc;
    % if t > 0.1
    %     disp(['出现问题，耗时' num2str(t*1000) 'ms'])
    % end
    pause(1);
end


cancel(f1)
% 更新共享字典的函数


function updateSharedDict(data, sharedDict)
    key = data{1};
    value = data{2};
    sharedDict(key) = value;
    disp('运行到这里啦')
end



    
vehicleID = 'f_2.2'

for i = 1:10000
    % if dq.QueueLength > 0
    %     % 从队列中获取数据
    %     [key, value] = receive(dq);
    % 
    %     % 更新共享字典
    %     sharedDict(key) = value;
    % end
    tic
    laneID = traci.vehicle.getLaneID(vehicleID);
    % traci.vehicle.getUniversal('0x51',vehicleID);
    
    % sharedDict
    t = toc;
    if t > 0.1
        disp(['出现问题，耗时' num2str(t*1000) 'ms'])
    end

    % pause(1);
end
