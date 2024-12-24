classdef LaneConnection_SUMO
    properties
        from
        to
        via
        dir
        state
    end
    methods
        function obj = LaneConnection_SUMO(varargin)
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