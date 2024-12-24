function [p_proj, dev, lineDist, refHead] = projPoint2Polyline(line, point) %#codegen
    % 输入参数说明：
    % line - 大小为 n x 2 的数组，表示折线的各个点
    % point - 大小为 1 x 2 的数组，表示要投影的点
    l_dist = xy2dist(line);
    % 初始化变量
    p_proj = line(1, :);
    min_dist = inf;
    dev = 0;
    lineDist = 0;
    refHead = [1,0];
    
    dist = sqrt((line(:,1)-point(1)).^2+(line(:,2)-point(2)).^2);
    [~, idx] = min(dist);
    for i = (idx-1):idx
        if i == 0
            continue
        elseif i == length(line)
            continue
        end
        % 获取当前线段的起点和终点
        p1 = line(i, :);
        p2 = line(i+1, :);
        
        % 计算线段向量和点到线段起点的向量
        v = p2 - p1;
        w = point - p1;
        
        % 计算投影点在当前线段上的投影参数t
        t = dot(w, v) / dot(v, v);
        
        if t < 0
            % 投影点在线段外，选择线段起点
            p = p1;
        elseif t > 1
            % 投影点在线段外，选择线段终点
            p = p2;
        else
            % 投影点在线段内，计算投影点坐标
            p = p1 + t * v;
        end
        
        % 计算投影点与目标点之间的距离
        d = norm(point - p);
        
        if d < min_dist
            min_dist = d;
            p_proj = p;
            
            % 计算偏移距离
            v_perp = [v(2), -v(1)]; % 线段的垂直向量
            v_perp = v_perp / norm(v_perp); % 规范化
            dev = dot(p - point, v_perp); % 偏移距离有符号
            lineDist = l_dist(i) + norm(p_proj-p1);
            refHead = v/norm(v);
        end
    end
end
