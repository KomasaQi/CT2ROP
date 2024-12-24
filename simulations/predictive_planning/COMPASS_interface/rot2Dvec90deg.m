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