%% 画图SUMO实体
% 在指定figureID的图上画，idList是一个cell的形式，包含所有要画图的实体id
% rangeLimit 是一个结构体
function  handle_dict = plot3SUMOentity(entity_dict,idList,figureID,plotReq) %#codegen
    if nargin < 4
        plotReq = [];
    end
    handle_dict = dictionary();
    if ~isempty(idList)
        figure(figureID)
        hold on
        idList = unique(idList);
        for i = 1:length(idList)
            id = idList{i};
            entity = entity_dict{id};
            if isa(entity,'Lane_SUMO')
                line = entity.shape;
                vec0 = line(2,:)-line(1,:);
                segment1 = offset2DCurve_forLane(line,(entity.width)/2,rot2Dvec90deg(vec0,'left'));
                segment2 = offset2DCurve_forLane(line,(entity.width)/2,rot2Dvec90deg(vec0,'right'));
                segment = [segment1;segment2(end:-1:1,:)];
                if isempty(plotReq)
                    plotReq = PlotReq('color',[1 1 1],'height',0);
                    if strncmp(id,':',1)
                        plotReq.faceAlpha = 0.2;
                    else
                        plotReq.faceAlpha = 0.7;
                    end
                end
                handle = fill3(segment(:,1),segment(:,2),plotReq.height*ones(length(segment),1),...
                    plotReq.color,'FaceAlpha',plotReq.faceAlpha,'EdgeColor',plotReq.edgeColor,...
                    'LineWidth',plotReq.lineWidth);
                handle_dict(id) = handle;
            elseif isa(entity,'Edge_SUMO')
                newIDlist = cell(entity_dict{id}.laneNum,1);
                for j = 1:entity_dict{id}.laneNum
                     newIDlist{j} = entity_dict{id}.getLaneID(j-1);
                end
                nextHandleDict = plot3SUMOentity(entity_dict,newIDlist,figureID,plotReq);
                handle_dict = unionDictionary(handle_dict,nextHandleDict);
            elseif isa(entity,'Junction_SUMO')
                if isempty(plotReq)
                    plotReq = PlotReq('color',[1 0 0],'height',-0.001,'faceAlpha',0.5);
                end
                handle = fill3(entity.shape(:,1),entity.shape(:,2),...
                    plotReq.height*ones(length(entity.shape),1),plotReq.color,'FaceAlpha',plotReq.faceAlpha);
                handle_dict(id) = handle;
            else
                error(['id ' id '不是支持的绘图类型，主人快检查一下嘛'])
            end
    
        end
    end

end


%{
曲线的形式需要是nx2的形式，向第一个点的指定方向偏移最近的一侧偏移
%}
function curve_offset = offset2DCurve_forLane(curve0,offset_val,direction)
    curve = [curve0(1,:);curve0;curve0(end,:)];
    p1 = curve(2,:);
    ref_norm = [direction(1),direction(2)]/norm(direction);
    curve_offset = zeros(size(curve0));
    data_length = (size(curve0,1)+1);
    for i = 2:data_length
        % 计算新的p2,l2
        p2 = curve(i+1,:);
        % 进行偏移计算
        if i == data_length
            current_norm = ref_norm;
        else
            current_norm = nearestNorm(p1,p2,ref_norm);
        end 
        offset_direct = (current_norm + ref_norm)/2;
        curve_offset(i-1,:) = p1 + offset_direct*offset_val/norm(offset_direct);
        % 更新l1,p1,p2,参考法线
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


% 将二维向量旋转90度
% dir为'left'或者'right'，分别表示逆时针和顺时针旋转
function vec = rot2Dvec90deg(vec0,dir)
    vec = zeros(size(vec0));
    if length(vec0)~=2
        error(['输入向量vec0的长度应该为2,实际为尺寸为' num2str(size(vec0))])
    else
        if strncmpi(dir,'l',1) || strncmpi(dir,'a',1)
            vec(1) = -vec0(2);
            vec(2) = vec0(1);
        elseif strncmpi(dir,'r',1) || strncmpi(dir,'c',1)
            vec(1) = vec0(2);
            vec(2) = -vec0(1);
        else
            error('未识别的方向指令')
        end
    end
end