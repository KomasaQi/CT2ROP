classdef Queue_Cell
    properties
        elements  % 存储队列元素的cell数组
        count     % 队列中元素的数量
    end
    
    methods
        % 构造函数：初始化一个空队列
        function obj = Queue_Cell()
            obj.elements = {};  % 初始化空cell数组
            obj.count = 0;      % 初始元素数量为0
        end
        function obj = set.elements(obj,value)
            obj.elements = value;
        end
        function obj = set.count(obj,value)
            obj.count = value;
        end
        % 入队操作：向队列尾部添加一个元素
        function obj = enqueue(obj, element)
            obj.count = obj.count + 1;          % 队列元素数量加1
            obj.elements{obj.count} = element;  % 将元素添加到队列尾部
        end
        
        % 出队操作：移除并返回队列头部的元素
        function [obj, element] = dequeue(obj)
            if obj.count == 0
                error('队列已空，无法出队！');
            end
            element = obj.elements{1};  % 获取队列头部元素
            obj.elements = obj.elements(2:end);  % 移除队列头部元素
            obj.count = obj.count - 1;   % 队列元素数量减1
        end
        
        % 修改队列中指定位置的元素值
        function obj = modifyElement(obj, index, newValue)
            if index < 1 || index > obj.count
                error('索引超出队列范围！');
            end
            obj.elements{index} = newValue;  % 修改指定位置的元素值
        end
        
        % 判断队列是否为空
        function isEmpty = empty(obj)
            isEmpty = (obj.count == 0);
        end
        
        % 返回队列中元素的数量
        function num = length(obj)
            num = obj.count;
        end
    end
end
