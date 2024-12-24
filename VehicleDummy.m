classdef VehicleDummy
    properties
        acc
        speed
        heading
        heading_cos_sin
        vehicle
        pos
        waypoints
        laneID
        edgeID
        lanePosition
        route
        routeIdx % 当前route的idx，如果在交叉口内就是刚刚驶离的edge的idx
        vehID = 'notSpecified'    % 车辆的sumo全局ID
        dev     % 侧向偏移
        changeLane % 是否正在换道 -1:右换道 0:不换道 1:左换道
        targetLaneIdx % 目标车道No
        nearJunction = false;
        v_des = 30
        isCruising = false
    end
    methods
        function obj = VehicleDummy(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function laneNo = getLaneNo(obj)
            lastUnderscoreIdx = find(obj.laneID=='_',1,'last');
            laneNo = str2double(obj.laneID(lastUnderscoreIdx+1:end));
        end
        function flag = isInJunction(obj)
            if strncmp(obj.laneID,':',1)
                flag = true;
            else
                flag = false;
            end
        end
        function laneID = getTargetLaneID(obj)
            laneID = [obj.edgeID '_' num2str(obj.targetLaneIdx)];
        end
    end
end