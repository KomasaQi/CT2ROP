% 用来检测两条有向折线是否相交，如果相交就找出交点,并计算两条折线分别，从起点出发，到交点的距离。
function [isIntersect, intersectPoint, distance1, distance2] = findIntersection(trajectory1, trajectory2)
    % trajectory1 和 trajectory2 是两个 Nx2 的矩阵，表示两条有向折线的轨迹
    % 初始化
    isIntersect = false;
    intersectPoint = [];
    distance1 = inf;
    distance2 = inf;

    % 向量化计算每条线段的向量
    vec1 = diff(trajectory1);
    vec2 = diff(trajectory2);

    % 预计算每段的起点
    start1 = trajectory1(1:end-1, :);
    start2 = trajectory2(1:end-1, :);
    
    % 计算两条线段的交点
    for i = 1:size(vec1, 1)
        for j = 1:size(vec2, 1)
            % 向量 p1p2 和 p3p4 的参数表示
            p1 = start1(i, :);
            p2 = start1(i, :) + vec1(i, :);
            p3 = start2(j, :);
            p4 = start2(j, :) + vec2(j, :);
            
            % 计算 t 和 u
            denom = (p4(2) - p3(2)) * (p2(1) - p1(1)) - (p4(1) - p3(1)) * (p2(2) - p1(2));
            if denom == 0
                continue; % 平行或共线
            end
            
            t = ((p3(1) - p1(1)) * (p4(2) - p3(2)) - (p3(2) - p1(2)) * (p4(1) - p3(1))) / denom;
            u = ((p3(1) - p1(1)) * (p2(2) - p1(2)) - (p3(2) - p1(2)) * (p2(1) - p1(1))) / denom;
            
            if t >= 0 && t <= 1 && u >= 0 && u <= 1
                % 计算交点
                intersectPoint = p1 + t * (p2 - p1);
                
                % 计算距离
                distance1 = norm(intersectPoint - trajectory1(1, :));
                distance2 = norm(intersectPoint - trajectory2(1, :));
                
                isIntersect = true;
                return;
            end
        end
    end
end