% 一个显式轨迹规划器
classdef ExplicitLTR_TrajPlanner < handle
    properties
        a6_mat
        b6_mat
        tf = 4
        range = [10/3.6 120/3.6; 20 110; -60, 60; -90/180*pi 90/180*pi]
        v0_seq
        x_seq
        y_seq
        Dth_seq
        resolution = 0.1
        
    end
    methods
        function obj = ExplicitLTR_TrajPlanner(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function plotLTRminTraj(obj,v0,x,y,Dth,figureID)
            if nargin < 5
                figure
            else
                figure(figureID)
            end
            [a6,b6] = get_a6b6(obj,v0,x,y,Dth);
            traj = obj.calcOneTraj([a6,b6],v0,x,y,Dth);
            traj_5dof = obj.calcOneTraj([0 0],v0,x,y,Dth);
            plot(traj_5dof(:,2),traj_5dof(:,3),'g',LineWidth=1)
            hold on 
            plot(traj(:,2),traj(:,3),'r',LineWidth=1)
            hold off
            
        end

        function [a6,b6] = get_a6b6(obj,v0,x,y,Dth)
            % v = obj.getProximatedSpd(v0,x,y,Dth);
            a6 = interpn(obj.v0_seq, obj.x_seq, obj.y_seq, obj.Dth_seq, obj.a6_mat, v0, x, y, Dth, 'linear');
            b6 = interpn(obj.v0_seq, obj.x_seq, obj.y_seq, obj.Dth_seq, obj.b6_mat, v0, x, y, Dth, 'linear');
        end

        function traj = calcOneTraj(obj,params,v0,x,y,Dth)
            % 曲线约束参数
            tf_ = obj.tf;

            x0 = 0;
            y0 = 0;
            v0x = v0;
            v0y = 0;
            a0x = 0;
            a0y = 0;
            xf = x;
            yf = y;
            vf = obj.getProximatedSpd(v0,x,y,Dth);
            vfx = vf*cos(Dth);
            vfy = vf*sin(Dth);
            afx = 0;afy = 0;

            % 待控制的自由度系数变量
            % a6 = -0.02;
            % b6 = -0.03;
            
            % 根据约束求解其他系数
            c1x = 1/(tf_^2)*((xf-x0) - v0x*tf_ - 1/2*a0x*tf_^2);
            c2x = 1/tf_*((vfx-v0x)-a0x*tf_);
            c3x = (afx - a0x);
            cx = [c1x c2x c3x]';
            
            c1y = 1/(tf_^2)*((yf-y0) - v0y*tf_ - 1/2*a0y*tf_^2);
            c2y = 1/tf_*((vfy-v0y)-a0y*tf_);
            c3y = (afy - a0y);
            cy = [c1y c2y c3y]';
            
            d = [1;6;30]*tf_^4;
            tfs = [tf_ tf_^2 tf_^3];
            
            A = [1 1 1; 3 4 5; 6 12 20].*tfs;
            A_cy = A\cy;
            A_cx = A\cx;
            A_d = A\d;
            b_ = [y0;v0y;1/2*a0y;A_cy;0];
            a_ = [x0;v0x;1/2*a0x;A_cx;0];
            % 剩余自由度系数
            a6 = params(1);
            b6 = params(2);
            a = [a_(1:3);a_(4:6)-A_d*a6;a6];
            b = [b_(1:3);b_(4:6)-A_d*b6;b6];
            % 生成曲线
            t = (0:obj.resolution:obj.tf)';
            traj = [t,polyval(a(end:-1:1),t),polyval(b(end:-1:1),t)];
            
        end
        function v = getProximatedSpd(obj,v0,x,y,Dth)
            dist0 = sqrt(x^2+y^2);
            devTh = abs(atan2(-y,x))/pi*180; % 这是最终位置和初始速度方向的偏角
            dist_1st = dist0*interp1([0,100],[1,1.3],devTh); % 认为这个偏角越大，实际轨迹比直线连线更长
            dist_2nd = dist_1st*interp1([0,100],[1,1.3],abs(Dth)); % 又用最终方向角和初始方向角的差进行加长轨迹修正
            v = 2*dist_2nd/obj.tf - v0;
        end
    end
end