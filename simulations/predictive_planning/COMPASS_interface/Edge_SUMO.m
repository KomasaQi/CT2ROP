classdef Edge_SUMO
    properties
        id 
        from
        to
        laneNum
    end
    methods
        function obj = Edge_SUMO(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function laneID = getLaneID(obj,index)
            laneID = [obj.id '_' num2str(round(index))];
        end
    end
end