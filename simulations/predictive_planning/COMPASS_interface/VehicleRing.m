classdef VehicleRing
    properties
        ring_handle_inner
        ring_handle_outer
        x_inner
        x_outer
        y_inner
        y_outer
        z_data
         % 定义颜色和透明度渐变
        c_data_inner
        c_data_outer
        height = 0.1;
        alpha_data_outer
        alpha_data_inner
        figureID 
        % 设置颜色映射为青色 
        map
 

        center = [0,0];

        num_points = 20;
        r_inner = 0.5;
        r_outer = 1;

        scale_factor = 1; % 可以整体缩放光环
        total_move_num = 6; % 全部动画的循环帧数（自定义的帧数）
        % current_move_num = 2; % 当前在第几步
        alpha_move = [0.5 1 1 0.8 0.3 0.1];
        size_move = [0.6 1 1.3 1.5 1.6 1.7];
    end
    methods
        function obj = VehicleRing(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
            obj.c_data_inner =[ones(1, obj.num_points),zeros(1,obj.num_points)]';
            obj.c_data_outer = [zeros(1,obj.num_points), ones(1, obj.num_points)]';
            obj.height = 0.05;
            obj.alpha_data_outer = [zeros(1,obj.num_points), 0.2*ones(1, obj.num_points)]';
            obj.alpha_data_inner = [0.5*ones(1, obj.num_points),zeros(1,obj.num_points)]';
            obj.map = [linspace(0,0.3,5)',linspace(1,1,5)',linspace(1,0.7,5)'];
            % 生成圆的坐标
            theta = linspace(0, 2*pi, obj.num_points);
            x_inner = obj.r_inner * cos(theta);
            y_inner = obj.r_inner * sin(theta);
            x_outer = obj.r_outer * cos(theta);
            y_outer = obj.r_outer * sin(theta);
            % 合并坐标，创建环形
            x = [x_outer, fliplr(x_inner)]';
            y = [y_outer, fliplr(y_inner)]';
            obj.z_data = obj.height*ones(size(x));
            obj.x_outer = obj.scale_factor*4*x;
            obj.y_outer = obj.scale_factor*4*y;
            obj.x_inner = obj.scale_factor*2*x;
            obj.y_inner = obj.scale_factor*2*y;
            
            if isempty(obj.figureID)
                figure
            else
                figure(obj.figureID)
            end
            hold on
            obj.ring_handle_outer = fill3(obj.x_outer+ obj.center(1),obj.y_outer+ obj.center(2),obj.z_data,...
                obj.c_data_outer, 'FaceColor', 'interp', ...
    'EdgeColor', 'none', 'FaceAlpha', 'interp', 'FaceVertexAlphaData', obj.alpha_data_outer);
            obj.ring_handle_inner = fill3(obj.x_inner+ obj.center(1),obj.y_inner+ obj.center(2),obj.z_data,...
                obj.c_data_inner, 'FaceColor', 'interp', ...
    'EdgeColor', 'none', 'FaceAlpha', 'interp', 'FaceVertexAlphaData', obj.alpha_data_inner);
            colormap(obj.map);
        end

        function delete(obj)
            delete(obj.ring_handle_outer);
            delete(obj.ring_handle_inner);
        end
        
        function showAnimation_at_step(obj,step,center)
            step = mod(step,obj.total_move_num)+1;
            new_xdata_outer = center(1) + [obj.x_outer(1:obj.num_points);obj.x_outer(obj.num_points+1:end)*obj.size_move(step)];
            new_ydata_outer = center(2) + [obj.y_outer(1:obj.num_points);obj.y_outer(obj.num_points+1:end)*obj.size_move(step)];
            set(obj.ring_handle_outer,'XData',new_xdata_outer,'YData',new_ydata_outer);

            new_xdata_inner = center(1) + [obj.x_inner(1:obj.num_points)*obj.size_move(step);obj.x_inner(obj.num_points+1:end)];
            new_ydata_inner = center(2) + [obj.y_inner(1:obj.num_points)*obj.size_move(step);obj.y_inner(obj.num_points+1:end)];
            new_alpha_data_inner = obj.alpha_data_inner*obj.alpha_move(step);
            set(obj.ring_handle_inner,'XData',new_xdata_inner,'YData',new_ydata_inner,'FaceVertexAlphaData',new_alpha_data_inner);

        end


    end
end