%{
曲线的形式需要是nx2的形式，向第一个点的指定方向偏移最近的一侧偏移
%}
function curve_offset = offset2DCurve_dir(curve0,offset_val,dir)
    v = curve0(2,:) - curve0(1,:);
    if strncmpi(dir,'l',1)
        direction = [-v(2),v(1)];
    elseif strncmpi(dir,'r',1)
        direction = [v(2),-v(1)];
    else
        direction = [];
    end
    curve = [curve0(1,:);curve0;curve0(end,:)];
    l1 = 0; % 计算l1初始值
    p1 = curve(2,:);
    ref_norm = direction/norm(direction);
    curve_offset = zeros(size(curve0));
    data_length = (size(curve0,1)+1);
    for i = 2:data_length
        % 计算新的p2,l2
        p2 = curve(i+1,:);
        l2 = norm(p1-p2);
        % 进行偏移计算
        if i == data_length
            current_norm = ref_norm;
        else
            current_norm = nearestNorm(p1,p2,ref_norm);
        end 
        offset_direct = (l2*current_norm + l1*ref_norm)/(l1+l2);
        curve_offset(i-1,:) = p1 + offset_direct*offset_val/norm(offset_direct);
        % 更新l1,p1,p2,参考法线
        l1 = l2;
        p1 = p2;
        ref_norm = current_norm;
    end
end

function norm_vec = nearestNorm(p1,p2,direction)
    v = p2 - p1; % 从p1指向p2的向量
    v_clk90 =  [-v(2),  v(1)];
    cos_th_clk90 = calcAngleCos(v_clk90,direction);
    if cos_th_clk90 > 0
        norm_vec = v_clk90/norm(v);
    else
        norm_vec = [ v(2), -v(1)]/norm(v);
    end
end

function cos_th = calcAngleCos(v1,v2)
    cos_th = dot(v1,v2)/norm(v1)/norm(v2);
end