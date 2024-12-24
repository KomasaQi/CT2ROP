classdef Dashboard_Speed
    properties
        pre_str_handle
        speed_handle
        post_str_handle
        route_navi_handle
        veh_pos_marker
        map_handle
        icon_handle

        iconAxes
        mapAxes
        
        smallMap
        iconPath = 'COMPASS_icon.png';
        figureID 
        
    end
    methods
        function obj = Dashboard_Speed(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
            if isempty(obj.figureID)
                figure
            else
                figure(obj.figureID)
            end
            hold on
            %dim =  [x, y, width, height] 相对于figure的归一化坐标
            obj.pre_str_handle = annotation('textbox', [0.72 0.86 0.1 0.1], 'String', 'Speed', 'Color',[0.9 0.95 1],'FitBoxToText', 'on',...
'BackgroundColor', 'none', 'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'none');
            obj.speed_handle = annotation('textbox', [0.825 0.83 0.2 0.15], 'String', '---', 'Color',[1 1 1],'FitBoxToText', 'on',...
'BackgroundColor', 'none', 'FontSize', 20, 'FontWeight', 'bold', 'EdgeColor', 'none');
            obj.post_str_handle = annotation('textbox', [0.9 0.86 0.1 0.1], 'String', 'km/h', 'Color',[0.9 0.95 1],'FitBoxToText', 'on',...
'BackgroundColor', 'none', 'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'none');
            obj.route_navi_handle = annotation('textbox', [0.25 0.86 0.5 0.1], 'String', '即将开始导航', 'Color',[0.9 0.95 1],'FitBoxToText', 'off',...
'BackgroundColor', 'none','FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'none','HorizontalAlignment','center');
            % 读取PNG图标
            [iconImg, ~, iconAlpha] = imread(obj.iconPath);
            
            
            
            % 创建固定位置的图标注释
            iconDim = [0.02 0 0.25 0.1]; % [x, y, width, height] 相对于figure的归一化坐标
            iconAnnotation = annotation('textbox', iconDim, 'FitBoxToText', 'off', 'EdgeColor', 'none');
            iconAnnotation.Position = iconDim; % 位置
            
            % 将图标添加到注释
            obj.iconAxes = axes('Position', iconDim, 'Units', 'normalized','HitTest','off');
            obj.icon_handle = imshow(iconImg, 'Parent', obj.iconAxes);
            set(obj.iconAxes, 'color', 'none', 'XColor', 'none', 'YColor', 'none'); % 去除坐标轴
            set(obj.icon_handle,'AlphaData',iconAlpha)


            % 创建固定位置的图标注释
            mapDim = [0.02 0.75 0.24 0.24]; % [x, y, width, height] 相对于figure的归一化坐标
            mapAnnotation = annotation('textbox', mapDim, 'FitBoxToText', 'off', 'EdgeColor', 'none');
            mapAnnotation.Position = mapDim; % 位置
            
            % 将图标添加到注释
            obj.mapAxes = axes('Position', mapDim, 'Units', 'normalized','HitTest','off');
            obj.map_handle= imshow(obj.smallMap.img, 'Parent', obj.mapAxes);
            hold on
            set(obj.mapAxes, 'color', 'none', 'XColor', 'none', 'YColor', 'none'); % 去除坐标轴
            obj.veh_pos_marker = fill([1,1,-1,-1],[-1,1,1,-1],'r','Parent', obj.mapAxes,'EdgeColor','none');
            hold off
            % set(obj.map_handle,'AlphaData',iconAlpha)

        end
        function refreshMap(obj,egoPos,egoHeading,radius)
            % 获取图像大小
            [imgHeight, imgWidth, ~] = size(obj.smallMap.img);
            
            % 计算坐标到像素的转换比例
            % xScale = imgWidth / (obj.smallMap.xlimit(2) - obj.smallMap.xlimit(1));
            % yScale = imgHeight / (obj.smallMap.ylimit(2) - obj.smallMap.ylimit(1));
            
            % 计算显示区域在图像中的像素坐标
            xCenterPixel = (egoPos(1) - obj.smallMap.xlimit(1)) * obj.smallMap.xScale;
            yCenterPixel = (obj.smallMap.ylimit(2) - egoPos(2)) * obj.smallMap.xScale;
            radiusPixelX = radius * obj.smallMap.xScale; % 假设 x 和 y 方向的比例相同
            % radiusPixelY = radius * obj.smallMap.yScale; % 假设 x 和 y 方向的比例相同
            
            % 计算显示区域的边界像素坐标
            xMinPixel = max(1, round(xCenterPixel - radiusPixelX));
            xMaxPixel = min(imgWidth, round(xCenterPixel + radiusPixelX));
            yMinPixel = max(1, round(yCenterPixel - radiusPixelX));
            yMaxPixel = min(imgHeight, round(yCenterPixel + radiusPixelX));
            localWidth = xMaxPixel - xMinPixel+1;
            localHeight = yMaxPixel - yMinPixel+1;
            
            carPos = [xCenterPixel-xMinPixel,yCenterPixel-yMinPixel];

            % 截取显示区域
            localArea = obj.smallMap.img(yMinPixel:yMaxPixel, xMinPixel:xMaxPixel, :);
            % 创建一个遮罩
            [X, Y] = meshgrid(1:localWidth, 1:localHeight);
            mask = sqrt((X - carPos(1)).^2 + (Y - carPos(2)).^2) <= radiusPixelX;
            [x1,y1]=RotatePolygon(carPos(1)+20*[2/3,-1/3,-1/3],carPos(2)+20*[0,1/3,-1/3],carPos,-egoHeading);
            set(obj.map_handle,'CData',localArea,'AlphaData',mask);
            set(obj.veh_pos_marker,'XData',x1,'YData',y1);
            set(obj.mapAxes,'XLim',[0 localWidth],'YLim',[0 localHeight])

        end
        function setSpeed(obj,velocity)
            speed = velocity*3.6;
            if speed >= 100
                set(obj.speed_handle,'Position',[0.815 0.83 0.2 0.15])
            elseif speed < 10
                set(obj.speed_handle,'Position',[0.835 0.83 0.2 0.15])
            else
                set(obj.speed_handle,'Position',[0.825 0.83 0.2 0.15])
            end
            set(obj.speed_handle,'String',num2str(round(speed))) % 注意单位，这里从m/s到km/h
        end
        function setNavigation(obj,dir,remainDist)
            dist = round(remainDist);
            if dist < 0
                distString = '0 m';
            elseif dist >= 1000
                distString = [num2str(round(dist/1000,1)) ' km'];
            else
                distString = [num2str(10*round(dist/10)) ' m'];
            end
            if strcmpi(dir,'g')
                if dist < 5
                    set(obj.route_navi_handle,'String','您已到达目的地附近~')
                else
                    set(obj.route_navi_handle,'String',['沿当前道路继续行驶' distString '到达目的地附近'])
                end
            else
                if strcmpi(dir,'s')
                    cmdString = '直行';
                elseif strcmpi(dir,'l')
                    cmdString = '左转';
                elseif strcmpi(dir,'r')
                    cmdString = '右转';
                elseif strcmpi(dir,'t')
                    cmdString = '调头';
                end
                if dist <= 15
                    set(obj.route_navi_handle,'String',cmdString)
                elseif dist < 200
                    set(obj.route_navi_handle,'String',['前方' distString '路口' cmdString])
                else
                    set(obj.route_navi_handle,'String',['沿当前道路继续行驶' distString '后' cmdString])
                end
            end

                
        end
        function delete(obj)
            delete(obj.pre_str_handle);
            delete(obj.speed_handle);
            delete(obj.post_str_handle);
            delete(obj.icon_handle);
            delete(obj.route_navi_handle);
            delete(obj.veh_pos_marker);
            delete(obj.map_handle);

        end
        function [x1,y1]=RotatePolygon(~,x,y,center,th)
            pointNum=length(x);
            if length(x)~=length(y)
                disp('顶点x,y坐标不一致，请检查一下')
            end
            x1=zeros(size(x));
            y1=zeros(size(y));
            for i=1:pointNum
                xc=x(i)-center(1);
                yc=y(i)-center(2);
                x1(i)=(xc*cos(th)-yc*sin(th))+center(1);
                y1(i)=(xc*sin(th)+yc*cos(th))+center(2);
            end
        
        end
    end
end