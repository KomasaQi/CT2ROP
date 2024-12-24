classdef Lane_SUMO
    properties
        id
        index
        speed
        length
        shape
        width = 3.2;
    end
    methods
        function obj = Lane_SUMO(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
            
        end
        function edgeID = getEdgeID(obj)
            % 使用正则表达式去除最后一个下划线及其后的数字
            edgeID = regexprep(obj.id, '_\d+$', '');
        end
       
    end

end