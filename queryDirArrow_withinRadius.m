function idList = queryDirArrow_withinRadius(dirArrowMap,center,radius,maxNum)
    distances = sqrt((dirArrowMap.xyMat(:,1)-center(1)).^2+(dirArrowMap.xyMat(:,2)-center(2)).^2);
    arrowIdxs = findMinNIndicesWithDistance(distances, maxNum, radius);
    idList = {dirArrowMap.NO_dict{arrowIdxs'}};
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
    if  N < 1
        error('N must greater than 1.');
    end
    
    
    if N <= length(vector)
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
    else
        indices = 1:length(vector);
    end
end