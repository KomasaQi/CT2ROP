function toJunctionID = getToJunctionID(entity_dict,edgeID)
    if strncmp(edgeID,':',1) %说明是Junction的内部edge
        % 创建正则表达式
        % pattern = '_.*'; % 匹配末尾的_后跟一个或多个数字，直到字符串结束
        % % 使用regexprep函数替换匹配的部分为空字符串
        % toJunctionID = regexprep(edgeID, pattern, '');
        lastUnderscoreIdx = find(edgeID=='_',1,'last');
        toJunctionID = edgeID(2:lastUnderscoreIdx-1);
    else % 是一个正常的边
        toJunctionID = entity_dict{edgeID}.to;
    end
end