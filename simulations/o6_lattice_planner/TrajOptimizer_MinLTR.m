classdef TrajOptimizer_MinLTR < handle %#codegen
    properties 
        startBoundary
        finalBoundary
        ltr_zeta = 0.7;
        ltr_omega_n = 2;
        resolution_default = 0.1;
        L = 5; % 车辆轴距
        delta_lim = 5; % 前轮转角限制，deg
        deltaRate_lim = 5; % 前轮转角角速度限制，deg/s
    end
    methods
        function obj = TrajOptimizer_MinLTR(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function [traj,minCost,LTR,optParams,traj_compare,cost_compare] = lctrajopt(obj,timeDuration,startBdry,finalBdry,resolution)
            if nargin < 5 
                resolution = obj.resolution_default;
            elseif isempty(resolution)
                resolution = obj.resolution_default;
            end
            if ~isa(startBdry,'TrajBoundaryCondition') || ~isa(finalBdry,'TrajBoundaryCondition')
                error('起始点/终止点约束必须为TrajBoundaryCondition类型对象')
            else
                obj.startBoundary = startBdry;
                obj.finalBoundary = finalBdry;
            end
            
            % 曲线约束参数
            tf = timeDuration;
            [x0,y0,v0x,v0y,a0x,a0y] = startBdry.getValues();
            [xf,yf,vfx,vfy,afx,afy] = finalBdry.getValues();

            % 待控制的自由度系数变量
            % a6 = -0.02;
            % b6 = -0.03;
            
            % 根据约束求解其他系数
            c1x = 1/(tf^2)*((xf-x0) - v0x*tf - 1/2*a0x*tf^2);
            c2x = 1/tf*((vfx-v0x)-a0x*tf);
            c3x = (afx - a0x);
            cx = [c1x c2x c3x]';
            
            c1y = 1/(tf^2)*((yf-y0) - v0y*tf - 1/2*a0y*tf^2);
            c2y = 1/tf*((vfy-v0y)-a0y*tf);
            c3y = (afy - a0y);
            cy = [c1y c2y c3y]';
            
            d = [1;6;30]*tf^4;
            tfs = [tf tf^2 tf^3];
            
            A = [1 1 1; 3 4 5; 6 12 20].*tfs;
            A_cy = A\cy;
            A_cx = A\cx;
            A_d = A\d;
            b = [y0;v0y;1/2*a0y;A_cy;0];
            a = [x0;v0x;1/2*a0x;A_cx;0];
 
            % 定义fmincon算法设置
            options = optimoptions('fmincon','Algorithm','sqp');
            [optParams,minCost]=fmincon(@(params)obj.lossfcn(params,tf,a,b,A_d,resolution),[0 0],[],[],[],[],[],[],[],options);
       
            traj = obj.calcOneTraj(optParams,tf,a,b,A_d,resolution);
            LTR = obj.confirmLTR(optParams,tf,a,b,A_d,resolution);
            traj_compare = obj.calcOneTraj([0 0],tf,a,b,A_d,resolution);
            cost_compare = obj.lossfcn([0,0],tf,a,b,A_d,resolution);
       
            
        end
        function [traj,minCost,LTR,optParams,traj_compare,cost_compare] = lctrajopt_mex(obj,timeDuration,startBdry,finalBdry,resolution)
            if nargin < 5 
                resolution = obj.resolution_default;
            elseif isempty(resolution)
                resolution = obj.resolution_default;
            end
            if ~isa(startBdry,'TrajBoundaryCondition') || ~isa(finalBdry,'TrajBoundaryCondition')
                error('起始点/终止点约束必须为TrajBoundaryCondition类型对象')
            else
                obj.startBoundary = startBdry;
                obj.finalBoundary = finalBdry;
            end
            
            % 曲线约束参数
            tf = timeDuration;
            [x0,y0,v0x,v0y,a0x,a0y] = startBdry.getValues();
            [xf,yf,vfx,vfy,afx,afy] = finalBdry.getValues();

            % 待控制的自由度系数变量
            % a6 = -0.02;
            % b6 = -0.03;
            
            % 根据约束求解其他系数
            c1x = 1/(tf^2)*((xf-x0) - v0x*tf - 1/2*a0x*tf^2);
            c2x = 1/tf*((vfx-v0x)-a0x*tf);
            c3x = (afx - a0x);
            cx = [c1x c2x c3x]';
            
            c1y = 1/(tf^2)*((yf-y0) - v0y*tf - 1/2*a0y*tf^2);
            c2y = 1/tf*((vfy-v0y)-a0y*tf);
            c3y = (afy - a0y);
            cy = [c1y c2y c3y]';
            
            d = [1;6;30]*tf^4;
            tfs = [tf tf^2 tf^3];
            
            A = [1 1 1; 3 4 5; 6 12 20].*tfs;
            A_cy = A\cy;
            A_cx = A\cx;
            A_d = A\d;
            b = [y0;v0y;1/2*a0y;A_cy;0];
            a = [x0;v0x;1/2*a0x;A_cx;0];
     
            % 定义fmincon算法设置
          
            options = optimoptions('fmincon','Algorithm','sqp');
            [optParams,minCost]=fmincon(@(params)lossfcn_6order_mex(params,tf, ...
                a,b,A_d,resolution,obj.L,obj.delta_lim,obj.deltaRate_lim, ...
                obj.ltr_zeta,obj.ltr_omega_n),[0 0],[],[],[],[],[],[],[],options);
            
            traj = obj.calcOneTraj(optParams,tf,a,b,A_d,resolution);
            LTR = obj.confirmLTR(optParams,tf,a,b,A_d,resolution);
            traj_compare = obj.calcOneTraj([0 0],tf,a,b,A_d,resolution);
            cost_compare = lossfcn_6order_mex([0,0],tf, ...
                a,b,A_d,resolution,obj.L,obj.delta_lim,obj.deltaRate_lim, ...
                obj.ltr_zeta,obj.ltr_omega_n);
           
         
            
        end

        function [traj,minCost,LTR,optParams,traj_compare,cost_compare] = lctrajopt_mex2(obj,timeDuration,startBdry,finalBdry,resolution)
            if nargin < 5 
                resolution = obj.resolution_default;
            elseif isempty(resolution)
                resolution = obj.resolution_default;
            end
            if ~isa(startBdry,'TrajBoundaryCondition') || ~isa(finalBdry,'TrajBoundaryCondition')
                error('起始点/终止点约束必须为TrajBoundaryCondition类型对象')
            else
                obj.startBoundary = startBdry;
                obj.finalBoundary = finalBdry;
            end
            
            % 曲线约束参数
            tf = timeDuration;
            [x0,y0,v0x,v0y,a0x,a0y] = startBdry.getValues();
            [xf,yf,vfx,vfy,afx,afy] = finalBdry.getValues();

            % 待控制的自由度系数变量
            % a6 = -0.02;
            % b6 = -0.03;
            
            % 根据约束求解其他系数
            c1x = 1/(tf^2)*((xf-x0) - v0x*tf - 1/2*a0x*tf^2);
            c2x = 1/tf*((vfx-v0x)-a0x*tf);
            c3x = (afx - a0x);
            cx = [c1x c2x c3x]';
            
            c1y = 1/(tf^2)*((yf-y0) - v0y*tf - 1/2*a0y*tf^2);
            c2y = 1/tf*((vfy-v0y)-a0y*tf);
            c3y = (afy - a0y);
            cy = [c1y c2y c3y]';
            
            d = [1;6;30]*tf^4;
            tfs = [tf tf^2 tf^3];
            
            A = [1 1 1; 3 4 5; 6 12 20].*tfs;
            A_cy = A\cy;
            A_cx = A\cx;
            A_d = A\d;
            b = [y0;v0y;1/2*a0y;A_cy;0];
            a = [x0;v0x;1/2*a0x;A_cx;0];
     
            % 定义fmincon算法设置
            % tic
            [optParams,minCost]=fmincon_6order_mex(tf,a,b,A_d,resolution, ...
                obj.L,obj.delta_lim,obj.deltaRate_lim,obj.ltr_zeta,obj.ltr_omega_n);
            % toc
            traj = obj.calcOneTraj(optParams,tf,a,b,A_d,resolution);
            LTR = obj.confirmLTR(optParams,tf,a,b,A_d,resolution);
            traj_compare = obj.calcOneTraj([0 0],tf,a,b,A_d,resolution);
            cost_compare = lossfcn_6order_mex([0,0],tf, ...
                a,b,A_d,resolution,obj.L,obj.delta_lim,obj.deltaRate_lim, ...
                obj.ltr_zeta,obj.ltr_omega_n);
        end

        function cost = lossfcn(obj,params,tf,a_,b_,A_d,resolution)
            a6 = params(1);
            b6 = params(2);
            
            spd_der = (1:6)';
            acc_der = [2 6 12 20 30]';

            a_spd = [a_(2:3);a_(4:6)-A_d*a6;a6].*spd_der;
            b_spd = [b_(2:3);b_(4:6)-A_d*b6;b6].*spd_der;
            
            a_acc = [a_(3);a_(4:6)-A_d*a6;a6].*acc_der;
            b_acc = [b_(3);b_(4:6)-A_d*b6;b6].*acc_der;
            
            time = (0:resolution:tf)';
            % 计算|| LTR ||∞
            dx = polyval(a_spd(end:-1:1),time);
            dy = polyval(b_spd(end:-1:1),time);
            ddx = polyval(a_acc(end:-1:1),time);
            ddy = polyval(b_acc(end:-1:1),time);
            
            ay = (dx.*ddy-dy.*ddx)./sqrt(dx.^2+dy.^2);
            kappa = ay./(dx.^2+dy.^2);
            delta = atan(kappa*obj.L)*180/pi;
            deltaRate = gradient(delta)./gradient(time);
            y0 = zeros(2,1); % LTR;dLTR
            % y0 = [-0.5,10];
            [~,y] = ode45(@(t,y)obj.ltr_2order_dyn(t, y, [time;1.5*tf], [ay;0], obj.ltr_zeta, obj.ltr_omega_n),(0:resolution:1.5*tf),y0);
            LTR = y(:,1);
            % ||LTR||∞为优化目标，利用罚函数施加前轮转角、转角速度约束。
            cost = max(abs(LTR))+20*max(max(abs(delta))-obj.delta_lim,0)+20*max(max(abs(deltaRate))-obj.deltaRate_lim,0);
        end

        function LTR = confirmLTR(obj,params,tf,a_,b_,A_d,resolution)
            a6 = params(1);
            b6 = params(2);
            
            spd_der = (1:6)';
            acc_der = [2 6 12 20 30]';

            a_spd = [a_(2:3);a_(4:6)-A_d*a6;a6].*spd_der;
            b_spd = [b_(2:3);b_(4:6)-A_d*b6;b6].*spd_der;
            
            a_acc = [a_(3);a_(4:6)-A_d*a6;a6].*acc_der;
            b_acc = [b_(3);b_(4:6)-A_d*b6;b6].*acc_der;
            
            time = (0:resolution:tf)';
            % 计算|| LTR ||∞
            dx = polyval(a_spd(end:-1:1),time);
            dy = polyval(b_spd(end:-1:1),time);
            ddx = polyval(a_acc(end:-1:1),time);
            ddy = polyval(b_acc(end:-1:1),time);
            ay = (dx.*ddy-dy.*ddx)./sqrt(dx.^2+dy.^2);
            y0 = zeros(2,1); % LTR;dLTR
            [~,y] = ode45(@(t,y)obj.ltr_2order_dyn(t, y, time, ay, obj.ltr_zeta, obj.ltr_omega_n),time,y0);
            LTR = y(:,1);

        end


        % 将LTR近似为侧向加速度ay的二阶惯性环节
        function dy = ltr_2order_dyn(~, t, y, ay_time, ays, zeta, omega_n)
            LTR = y(1);
            dLTR = y(2);
            ay = interp1(ay_time,ays,t);
            dy = zeros(2,1);
            dy(1) = dLTR;
            dy(2) = ay - 2*zeta*omega_n*dLTR - omega_n^2*LTR; 
        end

        function traj = calcOneTraj(~,params,tf,a_,b_,A_d,resolution)
            % 剩余自由度系数
            a6 = params(1);
            b6 = params(2);
            a = [a_(1:3);a_(4:6)-A_d*a6;a6];
            b = [b_(1:3);b_(4:6)-A_d*b6;b6];
            % 生成曲线
            t = (0:resolution:tf)';
            traj = [t,polyval(a(end:-1:1),t),polyval(b(end:-1:1),t)];
            
        end
    end
end