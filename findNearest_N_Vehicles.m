function [nearVehicleList,nearVehiclePos] = findNearest_N_Vehicles(vehicleList,egoID,center,N,farestDist)
    otherVehicleList = setdiff(vehicleList,{egoID});
    if isempty(otherVehicleList)
        nearVehicleList = [];
        nearVehiclePos = [];
    else % 如果存在他车
        if length(otherVehicleList) <= N % 如果周车少于设定的最多显示车辆
            nearVehicleList = otherVehicleList;
            nearVehiclePos = zeros(length(otherVehicleList),2);
            for i = 1:length(otherVehicleList)% 计算距离
                nearVehiclePos(i,:) = traci.vehicle.getPosition(otherVehicleList{i});
            end
        else % 周车很多，从其中选取距离自车最近的N辆
            vehilcePos = zeros(length(otherVehicleList),2);
            for i = 1:length(otherVehicleList)% 计算距离
                vehilcePos(i,:) = traci.vehicle.getPosition(otherVehicleList{i});
            end
            vehicleDist = sqrt((vehilcePos(:,1)-center(1)).^2+(vehilcePos(:,2)-center(2)).^2);
            idxs = findMinNIndicesWithDistance(vehicleDist, N, farestDist);
            nearVehicleList = otherVehicleList(idxs);
            nearVehiclePos = vehilcePos(idxs,:);
        end
    end

    
end


function indices = findMinNIndicesWithDistance(vector, N, farestDist)
    % 找出向量中值最小的N个元素的索引，且这些元素之间的距离都小于farestDist
    % vector: 输入向量
    % N: 需要找出的最小值的数量
    % farestDist: 元素之间的最大距离阈值
    % indices: 返回的索引向量

    % 检查输入向量是否为列向量
    if ~isscalar(vector)
        vector = vector(:);
    end

    % 检查N是否有效
    if N > length(vector) || N <= 0
        error('N must be between 1 and the length of the vector.');
    end

    % 对向量进行排序并获取索引
    [~, sortedIndices] = sort(vector);
    % 初始化索引数组
    indices = [];

    % 遍历排序后的索引，找出满足条件的N个元素
    for i = 1:N
        % 检查当前索引是否满足距离条件
        if i > 1
            if abs(vector(sortedIndices(i)) - vector(sortedIndices(i-1))) > farestDist
                % warning('Not enough elements within the specified distance.');
                break;
            end
        end
        % 添加满足条件的索引
        indices = unique([indices, sortedIndices(i)]);
    end
end