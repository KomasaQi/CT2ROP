classdef Connection_SUMO
    properties
        from
        to
        fromLane
        toLane
        via
        dir
        state
    end
    methods
        function obj = Connection_SUMO(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function fromLaneID = getFromLaneID(obj)
            fromLaneID = [obj.from '_' num2str(obj.fromLane)];
        end

        function toLaneID = getToLaneID(obj)
            toLaneID = [obj.to '_' num2str(obj.toLane)];
        end
    end
end