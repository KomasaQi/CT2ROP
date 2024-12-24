classdef TrajBoundaryCondition
    %TrajBoundaryCondition 本类用于规范化用于最小LTR轨迹优化的边界条件
    %   此处显示详细说明
    properties
        pos
        heading
        spd
        acc    
    end
    properties(Dependent)
        spdVec
        accVec
    end

    methods
        function obj = TrajBoundaryCondition(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end

        function [x,y,vx,vy,ax,ay] = getValues(obj)
            head_cos_sin = [cos(obj.heading),sin(obj.heading)];
            x = obj.pos(1);
            y = obj.pos(2);
            vx = head_cos_sin(1)*obj.spd;
            vy = head_cos_sin(2)*obj.spd;
            ax = head_cos_sin(1)*obj.acc;
            ay = head_cos_sin(2)*obj.acc;
        end
        function value = get.spdVec(obj)
            value = [cos(obj.heading),sin(obj.heading)]*obj.spd;
        end
        function value = get.accVec(obj)
            value = [cos(obj.heading),sin(obj.heading)]*obj.acc;
        end
    end
end