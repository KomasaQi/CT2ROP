% 裁剪显示在自制GUI上的COMPASS规划路径
function trimmed_route = getTrimmedRoute(route,center,radius,head_c_s,dev)
    xs = route(:,1);
    ys = route(:,2);
    % center_vec_x = xs - center(1);
    % center_vec_y = ys - center(2);
    % dist = sqrt(center_vec_x.^2+center_vec_y.^2);
    outofrange_x = xs<(center(1)-radius) | xs > (center(1)+radius);
    outofrange_y = ys<(center(2)-radius) | ys > (center(2)+radius); % 在BOX约束外的不要
    outofrange = outofrange_x & outofrange_y;
    camera_vec_x = xs - (center(1)+dev*head_c_s(1));
    camera_vec_y = ys - (center(2)+dev*head_c_s(2));
    behindCamera = (camera_vec_x*head_c_s(1) + camera_vec_y*head_c_s(2)) < 0; % 在相机外视野外的也不要
    
    retainIdxs = ~(outofrange | behindCamera); 
    trimmed_route = route(retainIdxs,:);
end