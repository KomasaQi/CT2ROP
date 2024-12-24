function result = parseStringToMatrix(str)
    % PARSESTRINGTOMATRIX Convert a string containing pairs of numbers separated
    % by commas and spaces to a matrix.
    %
    % Syntax: matrix = parseStringToMatrix(str)
    %
    % Input:
    %   str - A string containing pairs of numbers separated by commas and spaces.
    %
    % Output:
    %   matrix - A matrix where each row corresponds to a pair of numbers.
    if isempty(str)
        result = [];
    else
        % 使用 split 函数按空格分割字符串
        pairs = split(str, ' ');
        % 初始化一个空数组，用于存储结果
        result = zeros(size(pairs,1),2);
        
        % 遍历每一对数字
        for i = 1:size(result,1)
            % 使用 strsplit 函数按逗号分割字符串
            numbers = strsplit(pairs{i}, ',');
            % 转换为双精度浮点数并存储在结果数组中
            result(i,:) = str2double(numbers);
        end
    end

end