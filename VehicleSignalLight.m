classdef VehicleSignalLight
    properties
        brkHandle_left % 左侧刹车灯
        brkHandle_right % 右侧刹车灯
        tlHandle_left % 左侧转向灯
        tlHandle_right % 右侧转向灯
        brkShape_left = [-1.02 0.9 .6; -1.02 0.9 .5; -1.02 0.7 .5; -1.02 0.7 .6];
        brkShape_right = [-1.02 -0.9 .6; -1.02 -0.9 .5; -1.02 -0.7 .5; -1.02 -0.7 .6];
        tlShape_left = [-1.02 1 .6; -1.02 1 .5; -1.02 0.9 .5; -1.02 0.9 .6];
        tlShape_right = [-1.02 -1 .6; -1.02 -1 .5; -1.02 -0.9 .5; -1.02 -0.9 .6];
        L = 4.7; % 车辆几何尺寸
        W = 1.8;
        H = 1.4;
        figureID
        brkColor = [1 0 0]; % 红色
        unBrkColor = [0.3 0 0]; % 暗红色
        turnColor = [1 1 0]; % 黄色
        unTurnColor = [0.3 0.3 0]; % 暗黄色
        disp = [-1.15,0,0];
    end
    methods
        function obj = VehicleSignalLight(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
            s = [(obj.L)/2,(obj.W)/2,(obj.H)];
            obj.brkShape_left = obj.brkShape_left.*s+obj.disp;
            obj.brkShape_right = obj.brkShape_right.*s+obj.disp;
            obj.tlShape_left = obj.tlShape_left.*s+obj.disp;
            obj.tlShape_right = obj.tlShape_right.*s+obj.disp;

            if isempty(obj.figureID)
                figure
            else
                figure(obj.figureID)
            end
            hold on
            obj.brkHandle_left = obj.plotLight(obj.brkShape_left,obj.brkColor);
            obj.brkHandle_right = obj.plotLight(obj.brkShape_right,obj.brkColor);
            obj.tlHandle_left = obj.plotLight(obj.tlShape_left,obj.turnColor);
            obj.tlHandle_right = obj.plotLight(obj.tlShape_right,obj.turnColor);
        end

        function handle = plotLight(~,geometry,color)
            handle = fill3(geometry(:,1),geometry(:,2),geometry(:,3),color,'EdgeColor','none');
        end

        function refreshPosition(obj,veh_pos,heading)
            brk_l = obj.transform_geometry(obj.brkShape_left, heading, veh_pos);
            brk_r = obj.transform_geometry(obj.brkShape_right, heading, veh_pos);
            tl_l = obj.transform_geometry(obj.tlShape_left, heading, veh_pos);
            tl_r = obj.transform_geometry(obj.tlShape_right, heading, veh_pos);
            
            set(obj.brkHandle_left,'XData',brk_l(:,1),'YData',brk_l(:,2),'ZData',brk_l(:,3))
            set(obj.brkHandle_right,'XData',brk_r(:,1),'YData',brk_r(:,2),'ZData',brk_r(:,3))
            set(obj.tlHandle_left,'XData',tl_l(:,1),'YData',tl_l(:,2),'ZData',tl_l(:,3))
            set(obj.tlHandle_right,'XData',tl_r(:,1),'YData',tl_r(:,2),'ZData',tl_r(:,3))
        end

        function setLightState(obj,light_name,state)
            brkFlag = false;
            if strcmp(light_name,'brake_left')
                handle = obj.brkHandle_left;
                brkFlag = true;
            elseif strcmp(light_name,'brake_right')
                handle = obj.brkHandle_right;
                brkFlag = true;
            elseif strcmp(light_name,'turn_left')
                handle = obj.tlHandle_left;
            elseif strcmp(light_name,'turn_right')
                handle = obj.tlHandle_right;
            else
                error('未识别的灯类型，请使用brake, turn_left, turn_right其中一种')
            end
            if brkFlag
                if strcmp(state,'on')
                    color = obj.brkColor;
                else
                    color = obj.unBrkColor;
                end
            else
                if strcmp(state,'on')
                    color = obj.turnColor;
                else
                    color = obj.unTurnColor;
                end
            end
            if strcmp(state,'on')
                set(handle,"FaceColor",color,'FaceLighting', 'none', 'AmbientStrength', 1, 'DiffuseStrength', 0, 'SpecularStrength', 0);
            else
                set(handle,"FaceColor",color,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
            end
        end
        
        function setBrakeLight(obj,cmd)
            handle1 = obj.brkHandle_left;
            handle2 = obj.brkHandle_right;
            if strcmp(cmd,'on')
                color = obj.brkColor;
                set(handle1,"FaceColor",color,'FaceLighting', 'none', 'AmbientStrength', 1, 'DiffuseStrength', 0, 'SpecularStrength', 0);
                set(handle2,"FaceColor",color,'FaceLighting', 'none', 'AmbientStrength', 1, 'DiffuseStrength', 0, 'SpecularStrength', 0);
            else
                color = obj.unBrkColor;
                set(handle1,"FaceColor",color,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
                set(handle2,"FaceColor",color,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
                
            end
        end

        function setTrunLight(obj,cmd) % 'left','right','off'
            handle1 = obj.tlHandle_left;
            handle2 = obj.tlHandle_right;
            if strcmp(cmd,'left')
                set(handle1,"FaceColor",obj.turnColor,'FaceLighting', 'none', 'AmbientStrength', 1, 'DiffuseStrength', 0, 'SpecularStrength', 0);
                set(handle2,"FaceColor",obj.unTurnColor,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
            elseif strcmp(cmd,'right')
                set(handle2,"FaceColor",obj.turnColor,'FaceLighting', 'none', 'AmbientStrength', 1, 'DiffuseStrength', 0, 'SpecularStrength', 0);
                set(handle1,"FaceColor",obj.unTurnColor,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
            elseif strcmp(cmd,'off')
                set(handle1,"FaceColor",obj.unTurnColor,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
                set(handle2,"FaceColor",obj.unTurnColor,'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.9);
            else
                error('指令错误，请从left right off选择')
            end
        end

        function transformed_geometry = transform_geometry(~, geometry, heading, veh_pos)
            % 旋转矩阵（绕Z轴旋转）
            R = [cos(heading), -sin(heading), 0; 
                 sin(heading), cos(heading), 0; 
                 0, 0, 1];
        
            % 将几何体点进行旋转
            rotated_geometry = (R * geometry')';
        
            % 将几何体点进行平移（相对于全局坐标系的车辆位置）
            transformed_geometry = rotated_geometry + veh_pos;
        end
        function delete(obj)
            delete(obj.brkHandle_left)
            delete(obj.brkHandle_right)
            delete(obj.tlHandle_left)
            delete(obj.tlHandle_right)
        end

    end
end