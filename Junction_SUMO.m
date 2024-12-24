classdef Junction_SUMO
    properties
        id 
        pos
        type = 'dead_end';
        name
        x 
        y 
        incLanes
        intLanes
        shape
    end
    methods
        function obj = Junction_SUMO(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
    end
end