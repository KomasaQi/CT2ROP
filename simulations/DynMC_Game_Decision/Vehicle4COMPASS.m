classdef Vehicle4COMPASS %#codegen
    properties
        % 车辆几何参数
        L = 5;       % 车辆长度
        W = 2;       % 车辆宽度

        % idm参数
        a_max = 3;   % 最大加速度 
        b_comf= 2;   % 舒适减速度
        v_des = 30;  % 期望车速
        delta = 4;   % 加速度系数
        T_hw  = 1; % 期望跟车时距 1
        s_min = 3;   % 最小车间距（静止时）

        % mobil换道参数
        politenss = 0.2; % 礼让系数

        % idm参数
        sigma_a_max = 0.3;   % 最大加速度
        sigma_b_comf= 0.2;   % 舒适减速度
        sigma_v_des = 3;  % 期望车速
        sigma_delta = 0.4;   % 加速度系数
        sigma_T_hw  = 0.1; % 期望跟车时距
        sigma_s_min = 0.3;   % 最小车间距（静止时）

        % mobil换道参数
        sigma_politenss = 0.2; % 礼让系数

        % 车辆环境参数
        g = 9.806;   % 重力加速度
        mu = 0.8;    % 路面附着系数



        % 路线意图
        route       % 路线cell(routeNum,1)--->ID
        routeIdx    % 当前（创造场景时）在哪一段范围内
        routeNum    % 路线一共有多少段

        % routeNo

        % 路线状态
        laneID      % 创造场景时的车道ID
        edgeID      % 创造场景时的边ID

        
        
    end
    methods
        function obj = Vehicle4COMPASS(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function a = idm(obj,v_ego,v_p,s,v_des)
            dv = v_ego-v_p;
            s_star = obj.s_min + max(0, (v_ego*obj.T_hw + (v_ego*dv/(2*sqrt(obj.a_max*obj.b_comf)))));
            a = obj.a_max*(1-(v_ego/v_des)^obj.delta - (s_star/s)^2);
            a_cons = obj.g*obj.mu;
            if a > a_cons
                a = a_cons;
            elseif a < -a_cons
                a = -a_cons;
            end
        end

        % function [a, sigma2_a] = idm_w_ucty(obj,v_ego,v_p,s,v_des,sigma2_v_ego,sigma2_v_p,sigma2_v_des)
        %     a = obj.idm(v_ego,v_p,s,v_des);
        % 
        % end

        function flag = isRouteEnd(obj,routeIdx)
            if routeIdx >= obj.routeNum
                flag = true;
            else
                flag = false;
            end
        end
       
    end
end



