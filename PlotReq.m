classdef PlotReq
    properties
        height = 0;
        color = [0.85 0.9 0.9];
        faceAlpha = 1;
        lineWidth = 0.5;
        edgeColor = [0 0 0];
        
    end
    methods
        function obj = PlotReq(varargin)
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