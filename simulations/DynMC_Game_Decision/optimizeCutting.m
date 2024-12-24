function balancedSet = optimizeCutting(trimmedSet, M, ~)
    if nargin > 2
        ifDisp = true;
    else
        ifDisp = false;
    end
    % 木板的分段长度
    lengths = {[1 1 1 1 1 1 1 1 1 1 1 1 1], [9 8 7 6 5 4 4 3 3 2 1], [22 17 13 9 6 3 3 1 1]};
    mappings = [1:8 10:2:20];
    caseNum = length(trimmedSet);
    treeNameSet = cell(caseNum,1);
    for i = 1:caseNum
        treeNameSet{i} = trimmedSet{i};
    end
    % 初始化木板
    startIdxs = zeros(caseNum,1);
    finalIdxs = 17*ones(caseNum,1);
    accumIdxs = zeros(caseNum,1);
    boards = cellfun(@(i)length(i),trimmedSet,'un',0); % 场景的动作次数
    for i = 1:caseNum
        boards{i} = lengths{boards{i}}; 
    end
    
    % 需要切分的刀数
    cutsNeeded = M - length(boards);
    
    % 贪心算法切割木板
    cutBoards = cell(M,1);
    for i = 1:caseNum
        cutBoards{i} = boards{i}; 
    end
   

    % 对每块木板进行切割
    for i = 1:cutsNeeded
        boardLengths = cellfun(@sum, cutBoards);
        % sceNum = cellfun(@length,cutBoards);
        % moreboard
        [~, maxIdx] = max(boardLengths);
        % 从最长的木板中选择切割点
        longestBoard = cutBoards{maxIdx};
        possibleCuts = cumsum(longestBoard);
        
        % 在可以切的位置中，选择最接近平均值的一刀
        avgWorkload = sum(boardLengths) / M;
        [~, cutIdx] = min(abs(possibleCuts - avgWorkload));
        
        % 切割木板
        newBoard1 = longestBoard(1:cutIdx);
        newBoard2 = longestBoard(cutIdx+1:end);
        
        % 更新木板
        cutBoards{maxIdx} = newBoard1;
        accumIdxs(caseNum+i) = cutIdx + accumIdxs(maxIdx);
        startIdxs(caseNum+i) = cutIdx + accumIdxs(maxIdx);
        finalIdxs(caseNum+i) = finalIdxs(maxIdx);
        finalIdxs(maxIdx) = cutIdx  + accumIdxs(maxIdx);
        cutBoards{caseNum+i} = newBoard2; 
        treeNameSet{caseNum+i} = treeNameSet{maxIdx};
        
    end
    balancedSet = cell(M,1);
    needMapping = (startIdxs > 0);
    startIdxs(needMapping) = mappings(startIdxs(needMapping));
    needMapping = (finalIdxs < 17);
    finalIdxs(needMapping) = mappings(finalIdxs(needMapping));
    for i = 1:M
        balancedSet{i} = sprintf('%s_a%02db%02d',treeNameSet{i},startIdxs(i),finalIdxs(i));
    end
    balancedSet(startIdxs==finalIdxs) = [];
    if ifDisp
    % 分配给每个核心
    workloads = cellfun(@sum, cutBoards);
    % 打印结果
    fprintf('优化切分后的核心最大场景数量: %d\n', max(workloads));
    for i = 1:M
        if i <= length(balancedSet)
            fprintf('并行核心 %d: 负载场景数:%d, 轨迹构型:%s-a%02db%02d\n', i, workloads(i),treeNameSet{i},startIdxs(i),finalIdxs(i));
        else
            fprintf('并行核心 %d: 负载场景数:0\n', i);
        end
    end
    end

end

