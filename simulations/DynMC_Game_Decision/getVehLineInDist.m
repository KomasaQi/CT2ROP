% 给定一条折线，和在折线上的距离，寻找前方最远距离内的一条折线段
function line = getVehLineInDist(fullLine,lineDist,lineLen)
    dist = xy2dist(fullLine);

    [~,idx] = min(abs(dist-lineDist)); % 找到最近的点的序号
    if dist(idx) <= lineDist
        prev_idx = idx;
    else
        prev_idx = idx - 1;
    end
    if idx == length(dist)
        prev_idx = idx - 1;
    end
    prev_idx = max(prev_idx,1);
    p1 = fullLine(prev_idx,:);
    p2 = fullLine(prev_idx+1,:);
    inSecDist = lineDist - dist(prev_idx);
    v = p2 - p1;
    p_now = p1 + inSecDist/norm(v)*v;
    temp_line = [p_now;fullLine(prev_idx+1:end,:)];
    new_dist = xy2dist(temp_line);
    if new_dist(end) > lineLen % 如果剩下的距离很长，需要截取
        [~,idx] = min(abs(new_dist-lineLen)); % 找到最近的点的序号
        if new_dist(idx) <= lineLen
            prev_idx = idx;
        else
            prev_idx = idx - 1;
        end
        p1 = temp_line(prev_idx,:);
        p2 = temp_line(prev_idx+1,:);
        inSecDist = lineLen - new_dist(prev_idx);
        v = p2 - p1;
        p_end = p1 + inSecDist/norm(v)*v;
        line = [temp_line(1:idx,:);p_end];
    else
        line = temp_line;
    end
end